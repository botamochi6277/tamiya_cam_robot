#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import UInt32MultiArray
import time
import pigpio


class NeoPixel:

    def __init__(self, pi, n, channel=0, aux=0):
        self.pi = pi
        self.n = n
        self.buf = bytearray(self.n * 24)
        self.colors = [0] * self.n
        flag = 0
        if aux == 1:
            flag = 256
        self.h = self.pi.spi_open(channel, 6400000, flag)

        self.subscriber = rospy.Subscriber(
            '/neopixels', UInt32MultiArray, self.callback)
        self.message = UInt32MultiArray()

    def stop(self):
        for i in range(len(self.buf)):
            self.buf[i] = 0
        self.show()
        self.pi.spi_close(self.h)

    def set_color(self, i, color):
        if type(color) is list or type(color) is tuple:
            self.colors[i] = ((color[0] << 16) | (color[1] << 8) | color[2])
        else:
            self.colors[i] = color

    def fill(self, color):
        for i in range(self.n):
            self.set_color(i, color)

    def show(self):
        """Shows the new colors on the pixels themselves if they haven't already
        been autowritten.

        The colors may or may not be showing after this function returns because
        it may be done asynchronously."""
        for (i, c) in zip(range(self.n), self.colors):
            # RGB -> GRB
            d = ((c & 0xff0000) >> 8) | (
                (c & 0x00ff00) << 8) | ((c & 0x0000ff))
            # print('#{:06x}'.format(d))

            for j in range(24):
                # 2進数の1桁を抽出して比較
                if ((d >> (23 - j)) & 0b1) == 0b00:
                    self.buf[24 * i + j] = 0xE0
                else:
                    self.buf[24 * i + j] = 0xF8

        self.pi.spi_write(self.h, self.buf)
        time.sleep(100e-6)

    def callback(self, message):
        for i in range(len(message.data)):
            self.set_color(i, message.data[i])

        self.show()


def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('neopixel_listener', anonymous=True)

    pi = pigpio.pi()

    if not pi.connected:
        exit()
    num_pixels = 16
    pixels = NeoPixel(pi, n=num_pixels)
    rospy.loginfo('Please input neopixel colors as hex-color code')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
