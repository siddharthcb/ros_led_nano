#! /usr/bin/env python

import spidev
import sys
import time
import rospy
import rosparam
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from led_msgs.msg import led_blink, led_seq, led_solid, led_state


class SPItoWS:
    def __init__(self, ledc):

        rospy.init_node("adafruit_led")
        rospy.loginfo("running")

        #rospy.Subscriber("sbus_rc_ch", Int32MultiArray, self.rc_callback)
        rospy.Subscriber("led_solid", led_solid, self.led_solid_callback)
        rospy.Subscriber("led_seq", led_seq, self.led_seq_callback)
        rospy.Subscriber("led_blink", led_blink, self.led_blink_callback)

        self.led_state_pub = rospy.Publisher("LED_State", led_state, queue_size = 10)
        self.led_state = led_state()

        self.led_count = ledc
        self.X = '' # X is signal of WS281x
        for i in range(self.led_count):
            self.X = self.X + "100100100100100100100100100100100100100100100100100100100100100100100100"
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 2400000

        self.r = 0
        self.g = 0
        self.b = 0

        self.callback_timeout = 3.0 
        self.callback_timestamp = time.time()

        self.solid_state = False
        self.blink_state = False
        self.seq_state = False
        self.delay1 = 0
        self.delay2 = 0
        self.direction = 'cw'  #clockwise

        self.run()
        rospy.spin()

    def __del__(self):
        self.spi.close()

    def _Bytesto3Bytes(self, num, RGB): # num is number of signal, RGB is 8 bits (1 byte) str
        for i in range(8):
            if RGB[i] == '0':
                self.X = self.X[:num * 3 * 8 + i * 3] + '100' + self.X[num * 3 * 8 + i * 3 + 3:]
            elif RGB[i] == '1':
                self.X = self.X[:num * 3 * 8 + i * 3] + '110' + self.X[num * 3 * 8 + i * 3 + 3:]

    def _BytesToHex(self, Bytes):
        return ''.join(["0x%02X " % x for x in Bytes]).strip()

    def LED_show(self):
            Y = []
            for i in range(self.led_count * 9):
                Y.append(int(self.X[i*8:(i+1)*8],2))
            WS = self._BytesToHex(Y)
            self.spi.xfer3(Y, 2400000,0,8)

    def RGBto3Bytes_seq(self, led_num, R, G, B):

        if (R > 255 or G > 255 or B > 255):
            R, G, B = 255, 255, 255
        if (led_num > self.led_count - 1):
            led_num = 144
        RR = format(R, '08b')
        GG = format(G, '08b')
        BB = format(B, '08b')
        #for i in range(LED_COUNT):
        self._Bytesto3Bytes((led_num) * 3 , GG)
        self._Bytesto3Bytes((led_num) * 3 + 1, RR)
        self._Bytesto3Bytes(led_num * 3 + 2, BB)

    def RGBto3Bytes_solid(self, led_num, R, G, B):

        if (R > 255 or G > 255 or B > 255):
            R, G, B = 255, 255, 255
        if (led_num > self.led_count - 1):
            led_num = 144
        RR = format(R, '08b')
        GG = format(G, '08b')
        BB = format(B, '08b')
        for i in range(led_num):
            self._Bytesto3Bytes((i) * 3 , GG)
            self._Bytesto3Bytes((i) * 3 + 1, RR)
            self._Bytesto3Bytes(i * 3 + 2, BB)

    def LED_OFF_ALL(self):

        self.X = ''
        for i in range(self.led_count):
            self.X = self.X + "100100100100100100100100100100100100100100100100100100100100100100100100"
        self.LED_show()


    def led_solid_callback(self, msg):
        self.solid_state = msg.state

        self.led_count = msg.led_count
        self.r = msg.r
        self.g = msg.g
        self.b = msg.b

        self.blink_state = False
        self.seq_state = False

        self.callback_timestamp = time.time()

    
    def led_seq_callback(self, msg):
        self.seq_state = msg.state
        self.led_count = msg.led_count
        self.r = msg.r
        self.g = msg.g
        self.b = msg.b

        
        self.delay1 = msg.delay1
        self.delay2 = msg.delay2
        self.direction = msg.direction 

        self.blink_state = False
        self.solid_state = False

        self.callback_timestamp = time.time()

    def led_blink_callback(self, msg):
        self.blink_state = msg.state
        self.led_count = msg.led_count
        self.r = msg.r
        self.g = msg.g
        self.b = msg.b

        self.delay = msg.delay

        self.seq_state = False
        self.solid_state = False

        self.callback_timestamp = time.time()

    def running_leds(self, led_count, R, G, B, delay1, delay2):  #specify direction TO-DO

        #if self.direction == 'cw':   #clockwise direction

        for i in range(led_count):
            self.RGBto3Bytes_seq(i, R, G, B)
            print("showing leds")
            self.LED_show()
            time.sleep(delay1)

        time.sleep(delay2)    
        self.LED_OFF_ALL()

        '''elif self.direction == 'acw':

            for i in range(led_count):
                self.RGBto3Bytes_seq(i, R, G, B)
                print("showing leds")
                self.LED_show()
                time.sleep(delay1)

            time.sleep(delay2)    
            self.LED_OFF_ALL()'''

    def solid_color_leds(self, led_count, R, G, B):

        self.RGBto3Bytes_solid(led_count, R, G, B)
        self.LED_show()

    def blink_leds(self, led_count, R, G, B, delay):

        self.RGBto3Bytes_solid(led_count, R, G, B)
        self.LED_show()
        time.sleep(delay)
        self.LED_OFF_ALL()
        time.sleep(delay)

    def run(self):
    
        rate = rospy.Rate(50)      
  
        while not rospy.is_shutdown():

            if self.seq_state:
                self.running_leds(self.led_count, self.r, self.g, self.b, self.delay1, self.delay2)

                self.led_state.state = True
                self.led_state.mode = "sequential"
                self.led_state.r = self.r
                self.led_state.g = self.g
                self.led_state.b = self.b

            if self.solid_state:
                self.solid_color_leds(self.led_count, self.r, self.g, self.b)

                self.led_state.state = True
                self.led_state.mode = "solid"
                self.led_state.r = self.r
                self.led_state.g = self.g
                self.led_state.b = self.b

            if self.blink_state:
                self.blink_leds(self.led_count, self.r, self.g, self.b, self.delay)

                self.led_state.state = True
                self.led_state.mode = "blink"
                self.led_state.r = self.r
                self.led_state.g = self.g
                self.led_state.b = self.b



            elif not self.seq_state and not self.solid_state and not self.blink_state:
                self.LED_OFF_ALL()
                print("LEDs OFF")

                self.led_state.state = False
                self.led_state.mode = "LED OFF"
                self.led_state.r = 0
                self.led_state.g = 0
                self.led_state.b = 0

            if ((time.time() - self.callback_timestamp) > self.callback_timeout):
                self.LED_OFF_ALL()
                self.seq_state = False
                self.solid_state = False
                self.blink_state = False


            self.led_state_pub.publish(self.led_state)
        

            rate.sleep()

if __name__ == "__main__":
    LED_COUNT = 144
    
    sig = SPItoWS(LED_COUNT)

    