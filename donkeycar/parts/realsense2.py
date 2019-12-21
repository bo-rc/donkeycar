'''
Author: Tawn Kramer
File: realsense2.py
Date: April 14 2019
Notes: Parts to input data from Intel Realsense 2 cameras
'''
import time
import logging

import numpy as np
import pyrealsense2 as rs

class RS_T265(object):
    '''
    The Intel Realsense T265 camera is a device which uses an imu, twin fisheye cameras,
    and an Movidius chip to do sensor fusion and emit a world space coordinate frame that 
    is remarkably consistent.
    '''

    def __init__(self, stereo=False, image_output=True):
        #Using the image_output will grab two image streams from the fisheye cameras but return only one.
        #This can be a bit much for USB2, but you can try it. Docs recommend USB3 connection for this.
        self.stereo = stereo
        self.image_output = image_output

        # Declare RealSense pipeline, encapsulating the actual device and sensors
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)

        if self.image_output:
            #right now it's required for both streams to be enabled
            cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
            cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True
        
        zero_vec = (0.0, 0.0, 0.0)
        self.pos = zero_vec
        self.vel = zero_vec
        self.acc = zero_vec
        self.limg = None
        self.rimg = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.image_output:
            self.limg = np.asanyarray(frames.get_fisheye_frame(1).get_data())
            if self.stereo:
                self.rimg = np.asanyarray(frames.get_fisheye_frame(2).get_data())

        # Fetch pose frame
        pose = frames.get_pose_frame()

        if pose:
            data = pose.get_pose_data()
            self.pos = data.translation
            self.vel = data.velocity
            self.acc = data.acceleration
            logging.debug('realsense pos(%f, %f, %f)' % (self.pos.x, self.pos.y, self.pos.z))

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        if self.stereo:
            return self.limg, self.rimg
        else:
            return self.limg

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()


class RS_D435i(object):
    '''
    The Intel Realsense D435i camera is a RGBD camera with imu
    '''

    def __init__(self, image_w=320, image_h=240, frame_rate=30, img_type='color'):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        self.img_type = img_type

        if self.img_type == 'color':
            cfg.enable_stream(rs.stream.color, image_w, image_h, rs.format.bgr8, frame_rate)
        elif self.img_type == 'depth':
            cfg.enable_stream(rs.stream.depth, image_w, image_h, rs.format.z16, frame_rate)
        else:
            raise Exception("img_type >", img_type, "< not supported.")

        # Start streaming with requested config
        self.pipe.start(cfg)
        self.running = True
        
        self.img = None

    def poll(self):
        try:
            frames = self.pipe.wait_for_frames()
        except Exception as e:
            logging.error(e)
            return

        if self.img_type is 'color':
            self.img = np.asanyarray(frames.get_color_frame().get_data())
        elif self.img_type is 'depth':
            self.img = np.asanyarray(frames.get_depth_frame().get_data())
        else:
            raise Exception("img_type >", self.img_type, "< not supported.")

    def update(self):
        while self.running:
            self.poll()

    def run_threaded(self):
        return self.img

    def run(self):
        self.poll()
        return self.run_threaded()

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        self.pipe.stop()
