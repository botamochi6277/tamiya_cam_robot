#!/usr/bin/env python

# test-APA102.py
# 2017-03-28
# Public Domain

import time
import sys
import random
import rospy
from std_msgs.msg import UInt8MultiArray
import pigpio

LEDS = 30

WAVES = 0
PIGPIO = 1
SPIDEV = 2

DAT = 10
CLK = 11

DATB = (1 << DAT)
CLKB = (1 << CLK)

"""
Start Frame  0000 0000 | 0000 0000 | 0000 0000 | 0000 0000

LED Frame 1  111b bbbb | BBBB BBBB | GGGG GGGG | RRRR RRRR
...
LED Frame N  111b bbbb | BBBB BBBB | GGGG GGGG | RRRR RRRR

End Frame    1111 1111 | 1111 1111 | 1111 1111 | 1111 1111

b bbbb     Brightness 0-31
BBBB BBBB  Blue 0-255
GGGG GGGG  Green 0-255
RRRR RRRR  Red 0-255
"""

# https://answers.ros.org/question/225008/python-class-for-subscribing-data-for-several-topics/


class Pixels:

    def __init__(self, pi, method=PIGPIO):

        self.apa102_cmd = [0] * 4 + [0xe1, 0, 0, 0] * LEDS + [255] * 4
        self.chain = [None] * (2 * len(self.apa102_cmd))
        self.gwid = [None] * 16

        self.pi = pi
        self.sub = rospy.Subscriber(
            'colors', UInt8MultiArray, self.callback)

        if method == PIGPIO:
            # 0xE0 says not to set chip enables
            self.h = self.pi.spi_open(0, 2e6, 0xE0)
        else:
            self.oldDATmode = pi.get_mode(DAT)
            self.oldCLKmode = pi.get_mode(CLK)
            pi.set_mode(DAT, pigpio.OUTPUT)
            pi.set_mode(CLK, pigpio.OUTPUT)
            create_byte_waves()
        s = ''
        if method == WAVES:
            s += "pigpio waves"
        elif method == PIGPIO:
            s += "pigpio SPI"

        print(s)

    def __del__(self):

        for i in range(LEDS):
            set_LED_RGB(i, 0, 0, 0)
            update()

        if self.method == PIGPIO:
            self.pi.spi_close(self.h)
        else:
            while self.pi.wave_tx_busy():
                pass
            for w in self.gwid:
                self.pi.wave_delete(w)
            self.pi.set_mode(DAT, self.oldDATmode)
            self.pi.set_mode(CLK, self.oldCLKmode)

    def create_byte_waves(self):
        for i in range(16):
            pulse = []
            for bit in range(4):
                if (1 << (3 - bit)) & i:  # 1 bit
                    pulse.append(pigpio.pulse(DATB, CLKB, 1))
                else:  # 0 bit
                    pulse.append(pigpio.pulse(0, DATB | CLKB, 1))
                pulse.append(pigpio.pulse(CLKB, 0, 1))
            pi.wave_add_generic(pulse)
            gwid[i] = pi.wave_create()

    def tx_bytes(self, bytes):
        global chain
        while pi.wave_tx_busy():
            pass
        j = 0
        for i in range(len(bytes)):
            chain[j] = self.gwid[(bytes[i] >> 4) & 15]
            j += 1
            chain[j] = self.gwid[bytes[i] & 15]
            j += 1
        self.pi.wave_chain(chain)

    def update(self):
        if self.method == PIGPIO:
            self.pi.spi_xfer(self.h, self.apa102_cmd)
        else:
            self.tx_bytes(self.apa102_cmd)

    def set_LED_RGB(self, led, r, g, b):
        offset = (led * 4) + 4
        self.apa102_cmd[offset + 1] = b
        self.apa102_cmd[offset + 2] = g
        self.apa102_cmd[offset + 3] = r

    def set_LED_PRGB(self, led, p, r, g, b):
        offset = (led * 4) + 4
        self.apa102_cmd[offset] = 0xE0 + p
        self.apa102_cmd[offset + 1] = b
        self.apa102_cmd[offset + 2] = g
        self.apa102_cmd[offset + 3] = r

    def callback(self, array):

        # 0xFFAA00 <- 3Byte
        slots = 8
        # Calculate new position.

        # Clear old slots.
        for i in range(slots):
            if 0 <= i < LEDS:
                self.set_LED_RGB(i, 0, 0, 0)

        for i in range(slots):
            if 0 <= i < LEDS:
                # set_LED_RGB(ipos+i, (1<<i)+1, 0, 0)
                r = (array.data[i] & 0x00FF0000) >> 4
                g = (array.data[i] & 0x0000FF00) >> 2
                b = (array.data[i] & 0x000000FF)
                self.set_LED_RGB(i, r, g, b)

        # Refresh LEDs
        self.update()

        time.sleep(0.02)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('APA102_listener', anonymous=True)

    pi = pigpio.pi()

    if not pi.connected:
        exit()

    my_pix = Pixels(pi)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    pi.stop()

if __name__ == '__main__':
    listener()
