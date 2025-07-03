# SPDX-FileCopyrightText: 2020 FoamyGuy for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
Using time.monotonic() to blink the built-in LED.

Instead of "wait until" think "Is it time yet?"
"""
import time
import board
import neopixel

eye_pixel_pin = board.D9
eye_num_pixels = 2

eye_pixels = neopixel.NeoPixel(
    eye_pixel_pin, eye_num_pixels, brightness=0.2, auto_write=False)


# Define some colors
WHITE = (255, 255, 255)
OFF = (0, 0, 0)

def set_all_pixels(color):
    for i in range(eye_num_pixels):
        eye_pixels[i] = color
    eye_pixels.show()

def blink_pixels(color, delay_on, delay_off):
    """Blinks the NeoPixels with a specified color, on and off times."""
    now = time.monotonic()
    while True:
        if time.monotonic() - now <= delay_on:
            set_all_pixels(color)
        elif time.monotonic() - now <= delay_on + delay_off:
            set_all_pixels(OFF)
        else:
            now = time.monotonic()  # Reset timer

# Example usage: Blink green for 0.5 seconds, then off for 0.2 seconds
blink_pixels(WHITE, 12, 1)
