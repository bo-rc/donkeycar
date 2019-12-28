#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    pathplanner.py (drive) [--model=<model>]


Options:
    -h --help          Show this screen.
"""

import os
import time

from docopt import docopt
import numpy as np

import donkeycar as dk

#import parts
from donkeycar.parts.controller import JoystickController, get_js_controller, LocalWebControllerPlanner
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts.launch import AiLaunch
from donkeycar.parts.path import Path, PathPlot, CTE, CTE2, PID_Pilot, PlotPose, PImage, OriginOffset, PosStream, gray2color
from donkeycar.parts.transform import PIDController
from donkeycar.parts.datastore import TubHandler
from donkeycar.utils import *

def drive( cfg, model_path=None, meta=[] ):

    model_type = cfg.DEFAULT_MODEL_TYPE
    
    #Initialize car
    V = dk.vehicle.Vehicle()

    if cfg.RS_PATH_PLANNER:
        from donkeycar.parts.realsense2 import RS_T265
        print("RS t265 with path tracking")
        odometry = RS_T265(path_output=True,stereo_output=False, image_output=False)
        V.add(odometry,outputs=['rs/trans', 'rs/yaw'], threaded=True)
        V.add(PosStream(), inputs=['rs/trans', 'rs/yaw'], outputs=['pos/x', 'pos/y', 'pos/yaw'])
    else:
        raise Exception("This script is for RS_PATH_PLANNER only")

    from donkeycar.parts.realsense2 import RS_D435i
    cam = RS_D435i(img_type=cfg.RS_IMG_TYPE, image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, frame_rate=cfg.RS_FRAME_RATE)
    V.add(cam, inputs=[], outputs=['cam/image_array'], threaded=True)

    # class Distance:
    #     def run(self, img):
    #         h,w = img.shape
    #         arr = img[50:w-50,0:h-60].flatten()

    #         mean = np.mean(arr)

    #         print(mean)

    # V.add(Distance(),
    #     inputs=['cam/image_array'], outputs=['null'])
    
    ctr = get_js_controller(cfg)
    V.add(ctr, 
          inputs=['null'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
          threaded=True)

    #This web controller will create a web server
    web_ctr = LocalWebControllerPlanner()

    V.add(web_ctr,
          inputs=['map/image', 'cam/image_array'],
          outputs=['null'], # no part consumes this
          threaded=True)
        
    class UserCondition:
        def run(self, mode):
            if mode == 'user':
                return True
            else:
                return False

    V.add(UserCondition(), inputs=['user/mode'], outputs=['run_user'])
    
    class PilotCondition:
        def run(self, mode):
            if mode == 'user':
                return False
            else:
                return True       

    V.add(PilotCondition(), inputs=['user/mode'], outputs=['run_pilot'])

    path = Path(min_dist=cfg.PATH_MIN_DIST)
    V.add(path, inputs=['pos/x', 'pos/y'], outputs=['path'], run_condition='run_user')

    if os.path.exists(cfg.PATH_FILENAME):
        path.load(cfg.PATH_FILENAME)
        print("loaded path:", cfg.PATH_FILENAME)

    def save_path():
        path.save(cfg.PATH_FILENAME)
        print("saved path:", cfg.PATH_FILENAME)

    def clear_path():
        path.clear()
        print("path cleared, ready to record new path")

    ctr.set_button_down_trigger(cfg.SAVE_PATH_BTN, save_path)
    ctr.set_button_down_trigger(cfg.CLEAR_PATH_BTN, clear_path)

    img = PImage(resolution=(cfg.IMAGE_W, cfg.IMAGE_H), clear_each_frame=True)
    V.add(img, outputs=['map/image'])

    plot = PathPlot(scale=cfg.PATH_SCALE, offset=(cfg.IMAGE_W//4,cfg.IMAGE_H//2), color=(0,0,255))
    V.add(plot, inputs=['map/image', 'path'], outputs=['map/image'], run_condition='run_user')

    # this is error function for PID control
    V.add(CTE2(), inputs=['path', 'pos/x', 'pos/y'], outputs=['cte/error'], run_condition='run_pilot')

    pid = PIDController(p=cfg.PID_P, i=cfg.PID_I, d=cfg.PID_D)
    pilot = PID_Pilot(pid, cfg.PID_THROTTLE)
    V.add(pilot, inputs=['cte/error'], outputs=['pilot/angle', 'pilot/throttle'], run_condition="run_pilot")

    def dec_pid_p():
        pid.Kp -= 0.5
        print("pid: p- %f" % pid.Kp)

    def inc_pid_p():
        pid.Kp += 0.5
        print("pid: p+ %f" % pid.Kp)

    ctr.set_button_down_trigger("left_shoulder", dec_pid_p)
    ctr.set_button_down_trigger("right_shoulder", inc_pid_p)

    pos_plot = PlotPose(scale=cfg.PATH_SCALE, offset=(cfg.IMAGE_W//4,cfg.IMAGE_H//2))
    V.add(pos_plot, inputs=['map/image', 'pos/x', 'pos/y', 'pos/yaw'], outputs=['map/image'])

    if model_path:

        def load_model(kl, model_path):
            start = time.time()
            print('loading model', model_path)
            kl.load(model_path)
            print('finished loading in %s sec.' % (str(time.time() - start)) )
        
        #When we have a model, first create an appropriate Keras part
        kl = dk.utils.get_model_by_type(model_type, cfg)

        load_model(kl, model_path)

        outputs=['pilot/angle', 'pilot/throttle']

        V.add(kl, inputs=['cam/image_array'], 
            outputs=outputs,
            run_condition='run_pilot')          
    
    #Choose what inputs should change the car.
    class DriveMode:
        def run(self, mode, 
                    user_angle, user_throttle,
                    pilot_angle, pilot_throttle):
            if mode == 'user': 
                return user_angle, user_throttle
            
            elif mode == 'local_angle':
                return pilot_angle, user_throttle
            
            else:
                return pilot_angle, pilot_throttle * cfg.AI_THROTTLE_MULT
        
    V.add(DriveMode(), 
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle'], 
          outputs=['angle', 'throttle'])

    from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle

    steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    steering = PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM, 
                                    right_pulse=cfg.STEERING_RIGHT_PWM)
    
    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    throttle = PWMThrottle(controller=throttle_controller,
                                    max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                    zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                    min_pulse=cfg.THROTTLE_REVERSE_PWM)

    V.add(steering, inputs=['angle'])
    V.add(throttle, inputs=['throttle'])

    # data collection
    inputs=['cam/image_array',
            'user/angle', 'user/throttle', 
            'user/mode']

    types=['image_array',
           'float', 'float',
           'str']
    
    def get_record_alert_color(num_records):
        col = (0, 0, 0)
        for count, color in cfg.RECORD_ALERT_COLOR_ARR:
            if num_records >= count:
                col = color
        return col    

    class RecordTracker:
        def __init__(self):
            self.last_num_rec_print = 0
            self.dur_alert = 0
            self.force_alert = 0

        def run(self, num_records):
            if num_records is None:
                return 0
            
            if self.last_num_rec_print != num_records or self.force_alert:
                self.last_num_rec_print = num_records

                if num_records % 10 == 0:
                    print("recorded", num_records, "records")
                        
                if num_records % cfg.REC_COUNT_ALERT == 0 or self.force_alert:
                    self.dur_alert = num_records // cfg.REC_COUNT_ALERT * cfg.REC_COUNT_ALERT_CYC
                    self.force_alert = 0
                    
            if self.dur_alert > 0:
                self.dur_alert -= 1

            if self.dur_alert != 0:
                return get_record_alert_color(num_records)

            return 0

    V.add(RecordTracker(), inputs=["tub/num_records"], outputs=['records/alert'])

    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types, user_meta=meta)

    V.add(tub, inputs=inputs, outputs=["tub/num_records"], run_condition='recording')
    #tell the controller about the tub        
    ctr.set_tub(tub)
    
    ctr.print_controls()
    #run the vehicle
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        drive(cfg, model_path=args['--model'])