# The MIT License (MIT)
#
# Copyright (c) 2016 Damien P. George
# Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
#

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

            # WS2812
            #         0.35us   0.8us    (+-150ns)
            # 0:     |^^^^^|..........|
            #            0.7us   0.6us  (+-150ns)
            # 1:     |^^^^^^^^^^|.....|
            #
            # 8bit mode, 6400000Hz
            # t = 0.15625us
            #         0.35us   0.8us    (+-150ns)
            # 0:     |^^^|.....|
            #
            #            0.7us   0.6us  (+-150ns)
            # 1:     |^^^^|....|
            # 0:11100000,0xE0 -> [0.46875, 0.78125] us
            # 1:11110000,0xF0 -> [0.625, 0.625] us
            # 1:11111000,0xF8 -> [0.78125, 0.46875] us
            # MSB なので，上位ビットから変換
            for j in range(24):
                # 2進数の1桁を抽出して比較
                if ((d >> (23 - j)) & 0b1) == 0b00:
                    self.buf[24 * i + j] = 0xE0
                else:
                    self.buf[24 * i + j] = 0xF0

        self.pi.spi_write(self.h, self.buf)
        time.sleep(100e-6)


if __name__ == '__main__':

    pi = pigpio.pi()

    if not pi.connected:
        exit()
    num_pixels = 2
    pixels = NeoPixel(pi, n=num_pixels)

    def wheel(pos):
        # Input a value 0 to 255 to get a color value.
        # The colours are a transition r - g - b - back to r.
        if pos < 0 or pos > 255:
            r = g = b = 0
        elif pos < 85:
            r = int(pos * 3)
            g = int(255 - pos * 3)
            b = 0
        elif pos < 170:
            pos -= 85
            r = int(255 - pos * 3)
            g = 0
            b = int(pos * 3)
        else:
            pos -= 170
            r = 0
            g = int(pos * 3)
            b = int(255 - pos * 3)
        return (r, g, b)

    def rainbow_cycle(wait):
        for j in range(255):
            for i in range(num_pixels):
                pixel_index = (i * 256 // num_pixels) + j
                pixels.set_color(i, wheel(pixel_index & 255))
            pixels.show()
            time.sleep(wait)

    try:
        while True:
            # RGB test
            pixels.fill(0xFF0000)
            pixels.show()
            time.sleep(1)

            pixels.fill(0x00FF00)
            pixels.show()
            time.sleep(1)

            pixels.fill(0x0000FF)
            pixels.show()
            time.sleep(1)

            # Fading
            for i in range(255):
                # pixels.fill(i)
                pixels.set_color(0, (i, 0, 0))
                pixels.show()
                time.sleep(0.05)

            rainbow_cycle(0.01)    # rainbow cycle with 1ms delay per step

    except KeyboardInterrupt:
        pixels.fill(0x00)
        pixels.show()

        pixels.stop()

        pi.stop()
