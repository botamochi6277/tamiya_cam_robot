#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import UInt32MultiArray
import time
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

apa102_cmd = [0]*4 + [0xe1, 0, 0, 0]*LEDS + [255]*4
chain = [None]*(2*len(apa102_cmd))
gwid = [None]*16


class Dotstar:

    def __init__(self, n, method):
        # self.pi = pi
        self.n = n
        # self.buf = bytearray(self.n * 24)
        # self.colors = [0] * self.n
        # flag = 0
        # if aux == 1:
        #     flag = 256
        # self.h = self.pi.spi_open(channel, 6400000, flag)
        if method < WAVES or method > SPIDEV:
            method = WAVES
        self.method = method

        if method == PIGPIO or method == WAVES:
            self.pi = pigpio.pi()

            if not self.pi.connected:
                exit()

            if method == PIGPIO:
                # 0xE0 says not to set chip enables
                self.spi_handle = self.pi.spi_open(0, 2e6, 0xE0)
            else:
                self.oldDATmode = self.pi.get_mode(DAT)
                self.oldCLKmode = self.pi.get_mode(CLK)
                self.pi.set_mode(DAT, pigpio.OUTPUT)
                self.pi.set_mode(CLK, pigpio.OUTPUT)
                self.create_byte_waves()
        else:
            import spidev
            self.spi = spidev.SpiDev()
            self.spi.open(0, 0)
            self.spi.max_speed_hz = int(2e6)

        self.subscriber = rospy.Subscriber(
            '/dotstars', UInt32MultiArray, self.callback)
        self.message = UInt32MultiArray()

    def __del__(self):
        if self.method == PIGPIO or self.method == WAVES:
            if self.method == PIGPIO:
                self.pi.spi_close(self.spi_handle)
            else:
                while self.pi.wave_tx_busy():
                    pass
                for w in gwid:
                    self.pi.wave_delete(w)
                self.pi.set_mode(DAT, self.oldDATmode)
                self.pi.set_mode(CLK, self.oldCLKmode)
            self.pi.stop()
        else:
            self.spi.close()

    def create_byte_waves(self):
        for i in range(16):
            pulse = []
            for bit in range(4):
                if (1 << (3-bit)) & i:  # 1 bit
                    pulse.append(pigpio.pulse(DATB, CLKB, 1))
                else:  # 0 bit
                    pulse.append(pigpio.pulse(0, DATB | CLKB, 1))
                pulse.append(pigpio.pulse(CLKB, 0, 1))
            self.pi.wave_add_generic(pulse)
            gwid[i] = self.pi.wave_create()

    def tx_bytes(self, bytes):
        global chain
        while self.pi.wave_tx_busy():
            pass
        j = 0
        for i in range(len(bytes)):
            chain[j] = gwid[(bytes[i] >> 4) & 15]
            j += 1
            chain[j] = gwid[bytes[i] & 15]
            j += 1
        self.pi.wave_chain(chain)

    def update(self):
        if self.method == SPIDEV:
            self.spi.xfer(apa102_cmd)
        elif self.method == PIGPIO:
            self.pi.spi_xfer(self.spi_handle, apa102_cmd)
        else:
            self.tx_bytes(apa102_cmd)

    def set_LED_RGB(self, led, r, g, b):
        offset = (led*4) + 4
        apa102_cmd[offset+1] = b
        apa102_cmd[offset+2] = g
        apa102_cmd[offset+3] = r

    def set_LED_PRGB(self, led, p, r, g, b):
        offset = (led*4) + 4
        apa102_cmd[offset] = 0xE0 + p
        apa102_cmd[offset+1] = b
        apa102_cmd[offset+2] = g
        apa102_cmd[offset+3] = r

    def callback(self, message):
        # Clear LEDs.
        for i in range(LEDS):
            self.set_LED_RGB(i, 0, 0, 0)

        for i in range(len(message.data)):
            r = (message.data[i] & 0xff0000) >> 16
            g = (message.data[i] & 0x00ff00) >> 8
            b = (message.data[i] & 0x0000ff)
            self.set_LED_RGB(i, r, g, b)

        self.update()


def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('dotstar_listener', anonymous=True)

    num_pixels = 16
    pixels = Dotstar(num_pixels, 0)
    rospy.loginfo('Please input dotstar colors as hex-color code')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
