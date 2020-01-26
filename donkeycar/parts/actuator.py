"""
actuators.py
Classes to control the motors and servos. These classes 
are wrapped in a mixer class before being used in the drive loop.
"""

import time

import donkeycar as dk


import serial
from sys import version_info

PY2 = version_info[0] == 2   #Running Python 2.x?

#
#---------------------------
# Maestro Servo Controller
#---------------------------
#
# Support for the Pololu Maestro line of servo controllers
#
# Steven Jacobs -- Aug 2013
# https://github.com/FRC4564/Maestro/
#
# These functions provide access to many of the Maestro's capabilities using the
# Pololu serial protocol
#
class MaestroController:
    # When connected via USB, the Maestro creates two virtual serial ports
    # /dev/ttyACM0 for commands and /dev/ttyACM1 for communications.
    # Be sure the Maestro is configured for "USB Dual Port" serial mode.
    # "USB Chained Mode" may work as well, but hasn't been tested.
    #
    # Pololu protocol allows for multiple Maestros to be connected to a single
    # serial port. Each connected device is then indexed by number.
    # This device number defaults to 0x0C (or 12 in decimal), which this module
    # assumes.  If two or more controllers are connected to different serial
    # ports, or you are using a Windows OS, you can provide the tty port.  For
    # example, '/dev/ttyACM2' or for Windows, something like 'COM3'.
    def __init__(self,ttyStr='/dev/ttyACM0',device=0x0c):
        # Open the command port
        self.usb = serial.Serial(ttyStr)
        # Command lead-in and device number are sent for each Pololu serial command.
        self.PololuCmd = chr(0xaa) + chr(device)
        # Track target position for each servo. The function isMoving() will
        # use the Target vs Current servo position to determine if movement is
        # occuring.  Upto 24 servos on a Maestro, (0-23). Targets start at 0.
        self.Targets = [0] * 24
        # Servo minimum and maximum targets can be restricted to protect components.
        self.Mins = [0] * 24
        self.Maxs = [0] * 24
        
    # Cleanup by closing USB serial port
    def close(self):
        self.usb.close()

    # Send a Pololu command out the serial port
    def sendCmd(self, cmd):
        cmdStr = self.PololuCmd + cmd
        if PY2:
            self.usb.write(cmdStr)
        else:
            self.usb.write(bytes(cmdStr,'latin-1'))

    # Set channels min and max value range.  Use this as a safety to protect
    # from accidentally moving outside known safe parameters. A setting of 0
    # allows unrestricted movement.
    #
    # ***Note that the Maestro itself is configured to limit the range of servo travel
    # which has precedence over these values.  Use the Maestro Control Center to configure
    # ranges that are saved to the controller.  Use setRange for software controllable ranges.
    def setRange(self, chan, min, max):
        self.Mins[chan] = min
        self.Maxs[chan] = max

    # Return Minimum channel range value
    def getMin(self, chan):
        return self.Mins[chan]

    # Return Maximum channel range value
    def getMax(self, chan):
        return self.Maxs[chan]
        
    # Set channel to a specified target value.  Servo will begin moving based
    # on Speed and Acceleration parameters previously set.
    # Target values will be constrained within Min and Max range, if set.
    # For servos, target represents the pulse width in of quarter-microseconds
    # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
    # Typcially valid servo range is 3000 to 9000 quarter-microseconds
    # If channel is configured for digital output, values < 6000 = Low ouput
    def setTarget(self, chan, target):
        print("Maestro: ", chan, " target: ", target)
        # if Min is defined and Target is below, force to Min
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        # if Max is defined and Target is above, force to Max
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]
        #    
        lsb = target & 0x7f #7 bits for least significant byte
        msb = (target >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
        # Record Target value
        self.Targets[chan] = target
        
    # Set speed of channel
    # Speed is measured as 0.25microseconds/10milliseconds
    # For the standard 1ms pulse width change to move a servo between extremes, a speed
    # of 1 will take 1 minute, and a speed of 60 would take 1 second.
    # Speed of 0 is unrestricted.
    def setSpeed(self, chan, speed):
        lsb = speed & 0x7f #7 bits for least significant byte
        msb = (speed >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Set acceleration of channel
    # This provide soft starts and finishes when servo moves to target position.
    # Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
    # A value of 1 will take the servo about 3s to move between 1ms to 2ms range.
    def setAccel(self, chan, accel):
        lsb = accel & 0x7f #7 bits for least significant byte
        msb = (accel >> 7) & 0x7f #shift 7 and take next 7 bits for msb
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
    
    # Get the current position of the device on the specified channel
    # The result is returned in a measure of quarter-microseconds, which mirrors
    # the Target parameter of setTarget.
    # This is not reading the true servo position, but the last target position sent
    # to the servo. If the Speed is set to below the top speed of the servo, then
    # the position result will align well with the acutal servo position, assuming
    # it is not stalled or slowed.
    def getPosition(self, chan):
        cmd = chr(0x10) + chr(chan)
        self.sendCmd(cmd)
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb

    # Test to see if a servo has reached the set target position.  This only provides
    # useful results if the Speed parameter is set slower than the maximum speed of
    # the servo.  Servo range must be defined first using setRange. See setRange comment.
    #
    # ***Note if target position goes outside of Maestro's allowable range for the
    # channel, then the target can never be reached, so it will appear to always be
    # moving to the target.  
    def isMoving(self, chan):
        if self.Targets[chan] > 0:
            if self.getPosition(chan) != self.Targets[chan]:
                return True
        return False
    
    # Have all servo outputs reached their targets? This is useful only if Speed and/or
    # Acceleration have been set on one or more of the channels. Returns True or False.
    # Not available with Micro Maestro.
    def getMovingState(self):
        cmd = chr(0x13)
        self.sendCmd(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True

    # Run a Maestro Script subroutine in the currently active script. Scripts can
    # have multiple subroutines, which get numbered sequentially from 0 on up. Code your
    # Maestro subroutine to either infinitely loop, or just end (return is not valid).
    def runScriptSub(self, subNumber):
        cmd = chr(0x27) + chr(subNumber)
        # can pass a param with command 0x28
        # cmd = chr(0x28) + chr(subNumber) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    # Stop the current Maestro Script
    def stopScript(self):
        cmd = chr(0x24)
        self.sendCmd(cmd)

class MaestroSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1 
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       chan = 0,
                       left_pulse=4500,
                       right_pulse=7500):

        self.controller = controller
        self.chan = 0
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse


    def run(self, angle):
        #map absolute angle to angle that vehicle can implement.
        pulse = dk.utils.map_range(angle,
                                self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                self.left_pulse, self.right_pulse)

        self.controller.setTarget(self.chan, pulse)

    def shutdown(self):
        self.run(0) #set steering straight

class MaestroThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert throttle to PWM pulses.
    """
    MIN_THROTTLE = -1 
    MAX_THROTTLE = 1

    def __init__(self, controller=None,
                       min_pulse=3000,
                       max_pulse=9000):

        self.controller = controller
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse


    def run(self, throttle):
        #map absolute throttle to throttle that vehicle can implement.
        pulse = dk.utils.map_range(throttle,
                                self.MIN_THROTTLE, self.MAX_THROTTLE,
                                self.min_pulse, self.max_pulse)

        self.controller.setTarget(1, pulse)

    def shutdown(self):
        self.run(0) #set throttle neutral

class PCA9685:
    ''' 
    PWM motor controler using PCA9685 boards. 
    This is used for most RC Cars
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            #replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay) # "Tamiya TBLE-02" makes a little leap otherwise

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pulse(pulse)



class PiGPIO_PWM():
    '''
    # Use the pigpio python module and daemon to get hardware pwm controls from
    # a raspberrypi gpio pins and no additional hardware. Can serve as a replacement
    # for PCA9685.
    #
    # Install and setup:
    # sudo update && sudo apt install pigpio python3-pigpio
    # sudo systemctl start pigpiod
    #
    # Note: the range of pulses will differ from those required for PCA9685
    # and can range from 12K to 170K
    '''

    def __init__(self, pin, pgio=None, freq=75):
        import pigpio

        self.pin = pin
        self.pgio = pgio or pigpio.pi()
        self.freq = freq
        self.pgio.set_mode(self.pin, pigpio.OUTPUT)

    def __del__(self):
        self.pgio.stop()

    def set_pulse(self, pulse):
        self.pgio.hardware_PWM(self.pin, self.freq, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)


class JHat:
    ''' 
    PWM motor controler using Teensy emulating PCA9685. 
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        print("Firing up the Hat")
        import Adafruit_PCA9685
        LED0_OFF_L         = 0x08
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            #replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        self.register = LED0_OFF_L+4*channel

        #we install our own write that is more efficient use of interrupts
        self.pwm.set_pwm = self.set_pwm
        
    def set_pulse(self, pulse):
        self.set_pwm(self.channel, 0, pulse) 

    def set_pwm(self, channel, on, off):
        #print("pulse", off)
        """Sets a single PWM channel."""
        self.pwm._device.writeList(self.register, [off & 0xFF, off >> 8])
        
    def run(self, pulse):
        self.set_pulse(pulse)

class JHatReader:
    ''' 
    Read RC controls from teensy 
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        import Adafruit_PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.register = 0 #i2c read doesn't take an address
        self.steering = 0
        self.throttle = 0
        self.running = True
        #send a reset
        self.pwm._device.writeRaw8(0x06)

    def read_pwm(self):
        '''
        send read requests via i2c bus to Teensy to get
        pwm control values from last RC input  
        '''
        h1 = self.pwm._device.readU8(self.register)
        #first byte of header must be 100, otherwize we might be reading
        #in the wrong byte offset
        while h1 != 100:
            print("skipping to start of header")
            h1 = self.pwm._device.readU8(self.register)
        
        h2 = self.pwm._device.readU8(self.register)
        #h2 ignored now

        val_a = self.pwm._device.readU8(self.register)
        val_b = self.pwm._device.readU8(self.register)
        self.steering = (val_b << 8) + val_a
        
        val_c = self.pwm._device.readU8(self.register)
        val_d = self.pwm._device.readU8(self.register)
        self.throttle = (val_d << 8) + val_c

        #scale the values from -1 to 1
        self.steering = (((float)(self.steering)) - 1500.0) / 500.0  + 0.158
        self.throttle = (((float)(self.throttle)) - 1500.0) / 500.0  + 0.136
        
        #print(self.steering, self.throttle)

    def update(self):
        while(self.running):
            self.read_pwm()
            time.sleep(0.015)
        
    def run_threaded(self):
        return self.steering, self.throttle

    def shutdown(self):
        self.running = False
        time.sleep(0.1)


class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1 
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse


    def run(self, angle):
        #map absolute angle to angle that vehicle can implement.
        pulse = dk.utils.map_range(angle,
                                self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                self.left_pulse, self.right_pulse)

        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0) #set steering straight



class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        
        #send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.max_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.min_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)


    def run(self, throttle):
        if throttle > 0:
            pulse = dk.utils.map_range(throttle,
                                    0, self.MAX_THROTTLE, 
                                    self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.utils.map_range(throttle,
                                    self.MIN_THROTTLE, 0, 
                                    self.min_pulse, self.zero_pulse)

        self.controller.set_pulse(pulse)
        
    def shutdown(self):
        self.run(0) #stop vehicle



class Adafruit_DCMotor_Hat:
    ''' 
    Adafruit DC Motor Controller 
    Used for each motor on a differential drive car.
    '''
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
        import atexit
        
        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60) 
        
        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num
        
        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0
    
        
    def run(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        self.throttle = int(dk.utils.map_range(abs(speed), -1, 1, -255, 255))
        
        if speed > 0:            
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)
            
        self.motor.setSpeed(self.throttle)
        

    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)

class Maestro:
    '''
    Pololu Maestro Servo controller
    Use the MaestroControlCenter to set the speed & acceleration values to 0!
    '''
    import threading

    maestro_device = None
    astar_device = None
    maestro_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Maestro.maestro_device == None:
            Maestro.maestro_device = serial.Serial('/dev/ttyACM0', 115200)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Maestro.astar_device == None:
            Maestro.astar_device = serial.Serial('/dev/ttyACM2', 115200, timeout= 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds
        w *= 4  # in quarter microsenconds the maestro wants
        w = int(w)

        with Maestro.maestro_lock:
            Maestro.maestro_device.write(bytearray([ 0x84,
                                                     self.channel,
                                                     (w & 0x7F),
                                                     ((w >> 7) & 0x7F)]))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def readline(self):
        ret = None
        with Maestro.astar_lock:
            # expecting lines like
            # E n nnn n
            if Maestro.astar_device.inWaiting() > 8:
                ret = Maestro.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class Teensy:
    '''
    Teensy Servo controller
    '''
    import threading

    teensy_device = None
    astar_device = None
    teensy_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Teensy.teensy_device == None:
            Teensy.teensy_device = serial.Serial('/dev/teensy', 115200, timeout = 0.01)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Teensy.astar_device == None:
            Teensy.astar_device = serial.Serial('/dev/astar', 115200, timeout = 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds

        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("%c %.1f\n" % (self.channel, w)).encode('ascii'))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def teensy_readline(self):
        ret = None
        with Teensy.teensy_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.teensy_device.inWaiting() > 8:
                ret = Teensy.teensy_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

    def astar_readline(self):
        ret = None
        with Teensy.astar_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.astar_device.inWaiting() > 8:
                ret = Teensy.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass


class L298N_HBridge_DC_Motor(object):
    '''
    Motor controlled with an L298N hbridge from the gpio pins on Rpi
    '''
    def __init__(self, pin_forward, pin_backward, pwm_pin, freq = 50):
        import RPi.GPIO as GPIO
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.pwm_pin = pwm_pin

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_forward, GPIO.OUT)
        GPIO.setup(self.pin_backward, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.pwm_pin, freq)
        self.pwm.start(0)

    def run(self, speed):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        max_duty = 90 #I've read 90 is a good max
        self.throttle = int(dk.utils.map_range(speed, -1, 1, -max_duty, max_duty))
        
        if self.throttle > 0:
            self.pwm.ChangeDutyCycle(self.throttle)
            GPIO.output(self.pin_forward, GPIO.HIGH)
            GPIO.output(self.pin_backward, GPIO.LOW)
        elif self.throttle < 0:
            self.pwm.ChangeDutyCycle(-self.throttle)
            GPIO.output(self.pin_forward, GPIO.LOW)
            GPIO.output(self.pin_backward, GPIO.HIGH)
        else:
            self.pwm.ChangeDutyCycle(self.throttle)
            GPIO.output(self.pin_forward, GPIO.LOW)
            GPIO.output(self.pin_backward, GPIO.LOW)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm.stop()
        GPIO.cleanup()


class TwoWheelSteeringThrottle(object):

    def run(self, throttle, steering):
        if throttle > 1 or throttle < -1:
            raise ValueError( "throttle must be between 1(forward) and -1(reverse)")
 
        if steering > 1 or steering < -1:
            raise ValueError( "steering must be between 1(right) and -1(left)")

        left_motor_speed = throttle
        right_motor_speed = throttle
 
        if steering < 0:
            left_motor_speed *= (1.0 - (-steering))
        elif steering > 0:
            right_motor_speed *= (1.0 - steering)

        return left_motor_speed, right_motor_speed

    def shutdown(self):
        pass


class Mini_HBridge_DC_Motor_PWM(object):
    '''
    Motor controlled with an mini hbridge from the gpio pins on Rpi
    This can be using the L298N as above, but wired differently with only
    two inputs and no enable line.
    https://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Dtoys-and-games&field-keywords=Mini+Dual+DC+Motor+H-Bridge+Driver
    https://www.aliexpress.com/item/5-pc-2-DC-Motor-Drive-Module-Reversing-PWM-Speed-Dual-H-Bridge-Stepper-Motor-Mini
    '''
    def __init__(self, pin_forward, pin_backward, freq = 50, max_duty = 90):
        '''
        max_duy is from 0 to 100. I've read 90 is a good max.
        '''
        import RPi.GPIO as GPIO
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.max_duty = max_duty
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_forward, GPIO.OUT)
        GPIO.setup(self.pin_backward, GPIO.OUT)
        
        self.pwm_f = GPIO.PWM(self.pin_forward, freq)
        self.pwm_f.start(0)
        self.pwm_b = GPIO.PWM(self.pin_backward, freq)
        self.pwm_b.start(0)

    def run(self, speed):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed is None:
            return
        
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        self.throttle = int(dk.utils.map_range(speed, -1, 1, -self.max_duty, self.max_duty))
        
        if self.throttle > 0:
            self.pwm_f.ChangeDutyCycle(self.throttle)
            self.pwm_b.ChangeDutyCycle(0)
        elif self.throttle < 0:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(-self.throttle)
        else:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm_f.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.pwm_f.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def map_frange(self, x, X_min, X_max, Y_min, Y_max):
    ''' 
    Linear mapping between two ranges of values 
    '''
    X_range = X_max - X_min
    Y_range = Y_max - Y_min
    XY_ratio = X_range/Y_range

    y = ((x-X_min) / XY_ratio + Y_min)

    return y
    
class RPi_GPIO_Servo(object):
    '''
    Servo controlled from the gpio pins on Rpi
    '''
    def __init__(self, pin, freq = 50, min=5.0, max=7.8):
        import RPi.GPIO as GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.pin, freq)
        self.pwm.start(0)
        self.min = min
        self.max = max

    def run(self, pulse):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        #I've read 90 is a good max
        self.throttle = map_frange(pulse, -1.0, 1.0, self.min, self.max)
        #print(pulse, self.throttle)
        self.pwm.ChangeDutyCycle(self.throttle)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm.stop()
        GPIO.cleanup()


class ServoBlaster(object):
    '''
    Servo controlled from the gpio pins on Rpi
    This uses a user space service to generate more efficient PWM via DMA control blocks.
    Check readme and install here:
    https://github.com/richardghirst/PiBits/tree/master/ServoBlaster
    cd PiBits/ServoBlaster/user
    make
    sudo ./servod
    will start the daemon and create the needed device file:
    /dev/servoblaster

    to test this from the command line:
    echo P1-16=120 > /dev/servoblaster

    will send 1200us PWM pulse to physical pin 16 on the pi.

    If you want it to start on boot:
    sudo make install
    '''
    def __init__(self, pin):
        self.pin = pin
        self.servoblaster = open('/dev/servoblaster', 'w')
        self.min = min
        self.max = max

    def set_pulse(self, pulse):
        s = 'P1-%d=%d\n' % (self.pin, pulse)
        self.servoblaster.write(s)
        self.servoblaster.flush()

    def run(self, pulse):
        self.set_pulse(pulse)

    def shutdown(self):
        self.run((self.max + self.min) / 2)
        self.servoblaster.close()

