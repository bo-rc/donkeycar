"""
ArduinoPWM.py
Classes to interface with an Arduino board which
relays PWM signal via serial port running at baudrate 115200.

The Arduino sketch relays 2-channel PWM signals
separated by ',' and ended by a newline.

See Arduino sketch at:
https://github.com/bo-rc/ArduinoPWMRecorder

"""

import time
import serial

#
class ArduinoPWM:
    def __init__(self,port='/dev/ttyACM2',baud=115200, ch1_LOW = 1000, ch1_NEUTRAL = 1500, ch1_HIGH = 2000, 
    ch2_LOW = 1000, ch2_NEUTRAL = 1500, ch2_HIGH = 2000):
        # Open the command port
        print("open port for Arduino: ", port)
        self.usb = serial.Serial(port, baud, timeout = 0.1)
        self.counter = 0
        self.ch1 = 0.
        self.ch2 = 0.
        self.ch1_LOW = ch1_LOW
        self.ch1_HIGH = ch1_HIGH
        self.ch2_LOW = ch2_LOW
        self.ch2_HIGH = ch2_HIGH

        self.record = False

        self.buffer = ''
        

    def map_range(self, x, X_min, X_max, Y_min, Y_max):
        ''' 
        Linear mapping between two ranges of values 
        '''

        X_ratio = float(x - X_min) / float(X_max - X_min)
        Y_range = Y_max - Y_min

        y = X_ratio * Y_range + Y_min

        return y

    def update(self):
        self.buffer += self.usb.read(self.usb.inWaiting()).decode("utf-8")

        if '\n' in self.buffer:
            last_received, _ = self.buffer.split('\n')[-2:]
            self.buffer = ''
            
            data = last_received.strip().split(',')
            if len(data) > 2:

                ch1RAW = max(min(self.ch1_HIGH, int(data[0])), self.ch1_LOW)
                ch2RAW = max(min(self.ch2_HIGH, int(data[1])), self.ch2_LOW)

                self.ch1 = self.map_range(ch1RAW, self.ch1_LOW, self.ch1_HIGH, -1., 1.)
                self.ch2 = self.map_range(ch2RAW, self.ch2_LOW, self.ch2_HIGH, -1., 1.)

        # print("ch1: ", ch1RAW, self.ch1)
        # print("ch2: ", ch2RAW, self.ch2)

        if abs(self.ch2) > 0.05:
            self.record = True
        else:
            self.record = False
            
        return self.ch1, self.ch2, self.record

    def run(self):
        return self.run_threaded()

    def run_threaded(self):
        return self.update()

    def shutdown(self):
        self.usb.close()
        time.sleep(.5)

if __name__ == "__main__":
    iter = 0
    ard = ArduinoPWM(port='/dev/ttyACM2', baud=115200)
    while iter < 1000:
        ard.run_threaded()
        time.sleep(0.1)
        iter += 1
        print(1000 - iter, " left_> \n")
        print(ard.ch1, ",", ard.ch2, ", should record: ", ard.record)

    ard.shutdown()
