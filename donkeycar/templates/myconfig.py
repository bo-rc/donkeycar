# If desired, all config overrides can be specified here. 
# The update operation will not touch this file.
# """

DRIVE_LOOP_HZ = 21
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

RECORD_DURING_AI = False # shadow mode

# Maestro settings
DRIVE_TRAIN_TYPE = 'MAESTRO' # SERVO_ESC|DC_STEER_THROTTLE|DC_TWO_WHEEL|SERVO_HBRIDGE_PWM
MAESTRO_STEERING_LEFT_PWM = 4500 #pwm value for full left steering
MAESTRO_STEERING_RIGHT_PWM = 7500 #pwm value for full right steering
MAESTRO_THROTTLE_MIN_PWM = 4000 #pwm value for full throttle 
MAESTRO_THROTTLE_MAX_PWM = 8000 #pwm value for full throttle 
MAESTRO_THROTTLE_REVERSE = 0
MAESTRO_STERRING_REVERSE = 1

# use Arduino to record PWM
# as a better alternative to bluetooth JS
ARDUINO_PWM = 1

# Camera settings
CAMERA_TYPE = 'CSI_Stereo' # ('RS_T265_StereoRectified' | 'CSIC' | 'CSI_Stereo' | 'RS_T265' | 'RS_D435i')

CSIC_IMAGE_W = 224
CSIC_IMAGE_H = 224
CSIC_CAM_GSTREAMER_FLIP_PARM = 2 # rotate image 180

T265_IMAGE_W = 240
T265_IMAGE_H = 140
T265_FOV = 135

D435_IMAGE_W = 424 
D435_IMAGE_H = 240
D435_IMG_TYPE = "color"
D435_FRAME_RATE = DRIVE_LOOP_HZ

# this is for training resolution. 
# please be consistent with which camera is used.
IMAGE_W = CSIC_IMAGE_W
IMAGE_H = CSIC_IMAGE_H

# IMU in RealSense cameras
# use only with capable RS cameras
#USE_RS_IMU = False

# Waypoint navigation settings
SAVE_ROUTE_BTN = 'y_button'        # joystick button to save path
SAVE_WPT_BTN = 'x_button'
RS_PATH_PLANNER = True
RS_ROUTE_FILE = 'rs_route.pkl'
CLEAR_ROUTE_BTN = 'options'
PATH_MIN_DIST = 0.3                # after travelling this distance (m), save a path point
PATH_SCALE = 20
PID_P = -2.5                       # proportional mult for PID path follower
PID_I = 0.0                        # integral mult for PID path follower
PID_D = 0.0                        # differential mult for PID path follower
PID_THROTTLE = 0.3                 # constant throttle value during path following
WPT_TOLERANCE = 0.25               # meters

# PWM driver
PCA9685_I2C_BUSNUM = 1 

# #STEERING - Dakar - use PCA9685 only
STEERING_CHANNEL = 0            #channel on the 9685 pwm board 0-15
STEERING_LEFT_PWM = 250 #pwm value for full left steering
STEERING_RIGHT_PWM = 450 #pwm value for full right steering

# #THROTTLE - Dakar - use PCA9685 only
THROTTLE_CHANNEL = 1            #channel on the 9685 pwm board 0-15
THROTTLE_FORWARD_PWM = 450      #pwm value for max forward throttle
THROTTLE_STOPPED_PWM = 357      #pwm value for no movement
THROTTLE_REVERSE_PWM = 264      #pwm value for max reverse throttle

DEFAULT_MODEL_TYPE = 'linear'   #(linear|categorical|rnn|imu|motion|behavior|3d|localizer|latent)
AUTO_RECORD_ON_THROTTLE = True #if true, we will record whenever throttle is not zero. 
AI_THROTTLE_MULT = 0.5

CONTROLLER_TYPE='xbox'               #(ps3|ps4|xbox|nimbus|wiiu|F710|rc3)# import os
JOYSTICK_MAX_THROTTLE = 0.3
JOYSTICK_STEERING_SCALE = 1.0