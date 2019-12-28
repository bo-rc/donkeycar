# """ 
# My CAR CONFIG 

# This file is read by your car application's manage.py script to change the car
# performance

# If desired, all config overrides can be specified here. 
# The update operation will not touch this file.
# """

DRIVE_LOOP_HZ = 15

# #CAMERA
CAMERA_TYPE = "CSIC"   # (PICAM|WEBCAM|CVCAM|CSIC|V4L|MOCK)
IMAGE_W = 224
IMAGE_H = 224
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

# #RealSense CAMERA
#CAMERA_TYPE = "RS_D435i"   # (RS_D435i|RS_T265|RS_T265_StereoRectified|PICAM|WEBCAM|CVCAM|CSIC|V4L|MOCK)
#IMAGE_W = 320
#IMAGE_H = 240
#RS_FOV=120 
#RS_FRAME_RATE = DRIVE_LOOP_HZ
#RS_IMG_TYPE = "color"
#USE_RS_IMU = False
SAVE_PATH_BTN = 'y_button'         # joystick button to save path
CLEAR_PATH_BTN = 'options'
PATH_MIN_DIST = 0.3                # after travelling this distance (m), save a path point
PATH_SCALE = 20
PID_P = -5.0                       # proportional mult for PID path follower
PID_I = 2.500                       # integral mult for PID path follower
PID_D = -0.2                        # differential mult for PID path follower
PID_THROTTLE = 0.3                  # constant throttle value during path following

# PWM driver
PCA9685_I2C_BUSNUM = 1 

# #STEERING - Dakar
STEERING_CHANNEL = 0            #channel on the 9685 pwm board 0-15
STEERING_LEFT_PWM = 540         #pwm value for full left steering
STEERING_RIGHT_PWM = 250        #pwm value for full right steering

# #THROTTLE - Dakar
THROTTLE_CHANNEL = 1            #channel on the 9685 pwm board 0-15
THROTTLE_FORWARD_PWM = 480      #pwm value for max forward throttle
THROTTLE_STOPPED_PWM = 390      #pwm value for no movement
THROTTLE_REVERSE_PWM = 300      #pwm value for max reverse throttle

DEFAULT_MODEL_TYPE = 'rnn'   #(linear|categorical|rnn|imu|motion|behavior|3d|localizer|latent)

CONTROLLER_TYPE='xbox'               #(ps3|ps4|xbox|nimbus|wiiu|F710|rc3)# import os
AUTO_RECORD_ON_THROTTLE = False      #if true, we will record whenever throttle is not zero. 
                                     #if false, you must manually toggle recording with button b on xbox ctrl.

