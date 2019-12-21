# """ 
# My CAR CONFIG 

# This file is read by your car application's manage.py script to change the car
# performance

# If desired, all config overrides can be specified here. 
# The update operation will not touch this file.
# """

# #CAMERA
CAMERA_TYPE = "CSIC"   # (PICAM|WEBCAM|CVCAM|CSIC|V4L|MOCK)
IMAGE_W = 224
IMAGE_H = 224

PCA9685_I2C_BUSNUM = 1

# #STEERING - Dakar
STEERING_CHANNEL = 0            #channel on the 9685 pwm board 0-15
STEERING_LEFT_PWM = 540         #pwm value for full left steering
STEERING_RIGHT_PWM = 250        #pwm value for full right steering

# #THROTTLE - Dakar
THROTTLE_CHANNEL = 1            #channel on the 9685 pwm board 0-15
THROTTLE_FORWARD_PWM = 480 #pwm value for max forward throttle
THROTTLE_STOPPED_PWM = 390      #pwm value for no movement
THROTTLE_REVERSE_PWM = 300 #pwm value for max reverse throttle

DEFAULT_MODEL_TYPE = 'rnn'   #(linear|categorical|rnn|imu|behavior|3d|localizer|latent)

CONTROLLER_TYPE='xbox'               #(ps3|ps4|xbox|nimbus|wiiu|F710|rc3)