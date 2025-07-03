######################################################################
# Rosie the Radiant Rodent Control Code
#
# Install the following CircuitPython Libraries:
# - adafruit_bus_device
# - adafruit_fancyled
# - adafruit_led_animation
# - adafruit_motor
# - adafruit_debouncer.mpy
# - adafruit_ticks.mpy
# - neopixel.mpy
#
# Assumes the following hardware setup (pin assigments can be swapped to match
#  a different configuration)
# - Person Sensor on I2C bus (See https://usfl.ink/ps_dev for the full developer guide)
# - LED strings on Pins D10, D11, D12
# - Eyes on Pin 9
# - Capacative touch sensors on Pins D24, D25
# - 9g Servo Motor on Pin D13
#
# ####################################################################
# MIT License
# Debra Ansell
######################################################################


import board
import busio
import pwmio
import struct
import time
import random
import neopixel
from digitalio import DigitalInOut, Direction
from adafruit_debouncer import Debouncer
from adafruit_motor import servo
from adafruit_led_animation.animation.rainbow import Rainbow
from adafruit_led_animation.animation.comet import Comet
from adafruit_led_animation.animation.chase import Chase
from adafruit_led_animation.animation.rainbowchase import RainbowChase
from adafruit_led_animation.animation.rainbowcomet import RainbowComet
from adafruit_led_animation.animation.rainbowsparkle import RainbowSparkle
from adafruit_led_animation.color import colorwheel, RAINBOW
from adafruit_led_animation.sequence import AnimationSequence
from adafruit_led_animation.group import AnimationGroup


# Pin Assignments for LED strands in tail and ear
TAIL_PIN = board.D11
LEFT_EAR_PIN = board.D10
RIGHT_EAR_PIN = board.D12

# Number of pixels in each string
tail_pixels = 11
ear_pixels = 5

# Brightness index and vals used for changing brightness in response to input
brightness_index = 1
brightness_vals = [0.1, 0.3, 0.5, 0.7, 0.9]

# Create NeoPixel objects for the LED strings
tail = neopixel.NeoPixel(
    TAIL_PIN,
    tail_pixels,
    brightness=brightness_vals[brightness_index],
    auto_write=False,
)
left = neopixel.NeoPixel(
    LEFT_EAR_PIN,
    ear_pixels,
    brightness=brightness_vals[brightness_index],
    auto_write=False,
)
right = neopixel.NeoPixel(
    RIGHT_EAR_PIN,
    ear_pixels,
    brightness=brightness_vals[brightness_index],
    auto_write=False,
)

# Create NeoPixel Animations
EAR_PET_COLOR = (128, 0, 128)
default_speed = 0.07
left_comet = Comet(
    left, speed=0.05, color=(0, 255, 255), tail_length=4, bounce=False, reverse=True
)
right_comet = Comet(
    right, speed=0.05, color=(0, 255, 255), tail_length=4, bounce=False, reverse=True
)
left_chase = Chase(left, speed=0.08, color=EAR_PET_COLOR, size=1, spacing=1)
right_chase = Chase(right, speed=0.08, color=EAR_PET_COLOR, size=1, spacing=1)
rainbow = Rainbow(tail, speed=default_speed, period=1)
rainbow_comet = RainbowComet(tail, speed=default_speed, tail_length=5, bounce=False)
rainbow_sparkle = RainbowSparkle(tail, speed=default_speed, num_sparkles=5)

# Create animation sequence to easily switch between tail animations
animations = AnimationSequence(rainbow_comet, rainbow, rainbow_sparkle, auto_clear=True)
# Create animation group to coordinate animations between left and right ears
ear_chase_group = AnimationGroup(left_chase, right_chase)

# Identifies and fills the correct ear with the specified color
# depending on the x,y position of the detected face
def fill_ear_position(x, y, color=(0, 255, 0)):
    if x > 128:
        left.fill(color)
        left.show()
        right.fill((0, 0, 0))
        right.show()
    else:
        right.fill(color)
        right.show()
        left.fill((0, 0, 0))
        left.show()


# Fill both ears with the specified color
def fill_ears(color):
    left.fill(color)
    left.show()
    right.fill(color)
    right.show()


# Fill both ears and tail LED strips with specified color
def fill_all(color):
    fill_ears(color)
    tail.fill(color)
    tail.show()


# Fill the first num pixels in each ear with the specified color
def partial_ears(num, color):
    nfill = min(num, ear_pixels)
    for i in range(0, nfill):
        left[i] = color
        right[i] = color
    for i in range(nfill, ear_pixels):
        left[i] = (0, 0, 0)
        right[i] = (0, 0, 0)
    left.show()
    right.show()


# color_index = 0

# Set the brightness of all the LED strips to a discrete value specified
# in the array brightness_vals
def change_brightness():
    global brightness_index
    brightness_index = (brightness_index + 1) % len(brightness_vals)
    val = brightness_vals[brightness_index]
    tail.brightness = val
    left.brightness = val
    right.brightness = val
    fill_all((255, 255, 0))
    time.sleep(0.08)
    fill_all((0, 0, 0))


# Change the speed of the animations
def change_animation_speed(speed=default_speed):
    for a in animations._members:
        a.speed = speed


# EYES
eye_pixel_pin = board.D9
eye_num_pixels = 2

eye_pixels = neopixel.NeoPixel(
    eye_pixel_pin, eye_num_pixels, brightness=0.2, auto_write=False
)

WHITE = (255, 255, 255)
OFF = (0, 0, 0)


def set_all_pixels(color):
    for i in range(eye_num_pixels):
        eye_pixels[i] = color
    eye_pixels.show()


def blink_pixels(color, delay_on, delay_off):
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

# ------------------------------ Person Sensor Code ---------------------
# The person sensor has the I2C ID of hex 62, or decimal 98.
PERSON_SENSOR_I2C_ADDRESS = 0x62

# We will be reading raw bytes over I2C, and we'll need to decode them into
# data structures. These strings define the format used for the decoding, and
# are derived from the layouts defined in the developer guide.
PERSON_SENSOR_I2C_HEADER_FORMAT = "BBH"
PERSON_SENSOR_I2C_HEADER_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_I2C_HEADER_FORMAT)

PERSON_SENSOR_FACE_FORMAT = "BBBBBBbB"
PERSON_SENSOR_FACE_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_FACE_FORMAT)

PERSON_SENSOR_FACE_MAX = 4
PERSON_SENSOR_RESULT_FORMAT = (
    PERSON_SENSOR_I2C_HEADER_FORMAT
    + "B"
    + PERSON_SENSOR_FACE_FORMAT * PERSON_SENSOR_FACE_MAX
    + "H"
)
PERSON_SENSOR_RESULT_BYTE_COUNT = struct.calcsize(PERSON_SENSOR_RESULT_FORMAT)

# How long to pause between sensor polls.
PERSON_SENSOR_DELAY = 0.2

# ----------------------------Servo Code---------
# create a PWMOut object.
SERVO_PIN = board.D13
pwm = pwmio.PWMOut(SERVO_PIN, duty_cycle=2 ** 15, frequency=50)

# Create the head servo object
head_servo = servo.Servo(pwm)


LAST_MOVEMENT_TIME = -1
delay = random.random()

# Change these angles to change the center (rest_angle)
# and the max/min tilt of the head motions
# (min_angle, max_angle) of the range of motion
rest_angle = 90  # Position of servo, in degrees, when head is centered
tilt_offset = 20  # Max degrees of head motion to left or right
min_angle = rest_angle - tilt_offset
max_angle = rest_angle + tilt_offset
desired_angle = rest_angle

# Test Servo motion on startup
head_servo.angle = min_angle
time.sleep(0.5)
head_servo.angle = max_angle
time.sleep(0.5)
head_servo.angle = rest_angle
time.sleep(0.5)
# -------------------------End Servo--------

# Create random-seeming movements of the head servo
# scale changes the amplitude of the movements
def random_motion(scale=1.0):
    global LAST_MOVEMENT_TIME, delay
    global desired_angle

    now = time.monotonic()
    if now - LAST_MOVEMENT_TIME < delay:
        return

    delay = random.random()
    LAST_MOVEMENT_TIME = now

    low = int(
        min_angle if scale >= 1 else int(-(rest_angle - min_angle) * scale) + rest_angle
    )
    high = int(
        max_angle if scale >= 1 else int((max_angle - rest_angle) * scale) + rest_angle
    )
    desired_angle = random.randint(low, high)
    head_servo.angle = desired_angle


# Shake the rodent head back and forth for a specified time
# Triggered in response to patting the head touch sensor
shake_time_length = 1.6  # How many seconds the back-forth head motion lasts
shake_interval = 0.2  # Amount of time to give servo to complete one head tilt
shake_start = -1
shake_head = False
last_shake_time = 0
shake_pos_list = [min_angle, max_angle]
shake_pos = 0


def do_head_shake():
    global shake_start, last_shake_time, LAST_MOVEMENT_TIME
    global shake_pos, shake_head, delay, shake_time_length
    now = time.monotonic()

    # Just starting shake movement
    if shake_start < 0:
        shake_start = now
        shake_time_length = 1 + random.random()
    elif now - shake_start > shake_time_length:
        shake_head = False
        shake_start = -1
        head_servo.angle = rest_angle
        LAST_MOVEMENT_TIME = now
        fill_ears((0, 0, 0))
        return

    # Time elapsed for shake movement
    if now - last_shake_time > shake_interval:
        # print(now, shake_start, last_shake_time, shake_interval, shake_pos_list[shake_pos])
        head_servo.angle = shake_pos_list[shake_pos]
        shake_pos = (shake_pos + 1) % len(shake_pos_list)
        last_shake_time = now
        LAST_MOVEMENT_TIME = now
        delay = 1 + random.random()


# Pins corresponding to touch sensors in head and paw
TOUCH_PIN_HEAD = board.D24
TOUCH_PIN_PAW = board.D25
# --------------------------Touch Sensor Read-----
touch_sensor_head = DigitalInOut(TOUCH_PIN_HEAD)
touch_sensor_head.direction = Direction.INPUT
touch_switch_head = Debouncer(touch_sensor_head)

touch_sensor_paw = DigitalInOut(TOUCH_PIN_PAW)
touch_sensor_paw.direction = Direction.INPUT
touch_switch_paw = Debouncer(touch_sensor_paw)
# --------------end sensor read -----------------

# The Pico doesn't support board.I2C(), so check before calling it. If it isn't
# present then we assume we're on a Pico and call an explicit function.
try:
    i2c = board.STEMMA_I2C()
except:
    i2c = busio.I2C(scl=board.SCL1, sda=board.SDA1)

# Wait until we can access the bus.
while not i2c.try_lock():
    pass

# NONE/LEFT/RIGHT indicate which ear is illuminated
NONE = 0
LEFT = 1
RIGHT = 2

last_face_time = 0
fill_ears((0, 0, 0))

# Pretty ears indicates displaying an animated color pattern
# in the side of the ear where the closest face is detected
face_side = NONE


while True:
    now = time.monotonic()
    touch_switch_head.update()
    touch_switch_paw.update()

    # Paw was touched, advance to the next tail animation
    if touch_switch_paw.fell:
        animations.next()

    # Head was touched, shake head back and forth while running
    # a chase pattern in both ears
    if touch_switch_head.fell:
        # print("head")
        shake_head = True

    # Run tail animations
    animations.animate()

    if shake_head:  # Head animation if head sensor was touched
        do_head_shake()
        ear_chase_group.animate()
    elif face_side:  # Otherwise animate ear matching side of face detected
        ear_animation.animate()

    if (
        now - last_face_time > PERSON_SENSOR_DELAY
    ):  # Check periodically for face detection
        # if False:
        last_face_time = now
        read_data = bytearray(PERSON_SENSOR_RESULT_BYTE_COUNT)
        i2c.readfrom_into(PERSON_SENSOR_I2C_ADDRESS, read_data)

        offset = 0
        (pad1, pad2, payload_bytes) = struct.unpack_from(
            PERSON_SENSOR_I2C_HEADER_FORMAT, read_data, offset
        )
        offset = offset + PERSON_SENSOR_I2C_HEADER_BYTE_COUNT

        (num_faces) = struct.unpack_from("B", read_data, offset)
        num_faces = int(num_faces[0])
        offset = offset + 1

        faces = []
        if num_faces == 0:
            fill_ears((0, 0, 0))
            face_side = NONE
        for i in range(num_faces):
            (
                box_confidence,
                box_left,
                box_top,
                box_right,
                box_bottom,
                id_confidence,
                id,
                is_facing,
            ) = struct.unpack_from(PERSON_SENSOR_FACE_FORMAT, read_data, offset)
            offset = offset + PERSON_SENSOR_FACE_BYTE_COUNT
            face = {
                "box_confidence": box_confidence,
                "box_left": box_left,
                "box_top": box_top,
                "box_right": box_right,
                "box_bottom": box_bottom,
                "id_confidence": id_confidence,
                "id": id,
                "is_facing": is_facing,
            }
            faces.append(face)
        checksum = struct.unpack_from("H", read_data, offset)

        # If we've found any faces, the largest should be the first in the list, so
        # use that to determine which side of center the closest face is on
        if num_faces > 0:
            main_face = faces[0]

            # Center of closest face bounding box
            face_center_x = (main_face["box_left"] + main_face["box_right"]) / 2
            face_center_y = (main_face["box_top"] + main_face["box_bottom"]) / 2

            # Figure out which ear to light depending on which side of midline
            # the closest face appearts
            fs = LEFT if face_center_x > 128 else RIGHT
            if fs != face_side:
                face_side = fs
                if fs == LEFT:
                    ear_animation = left_comet
                    right.fill((0, 0, 0))
                    right.show()
                else:
                    ear_animation = right_comet
                    left.fill((0, 0, 0))
                    left.show()

        elif (
            not shake_head
        ):  # No face detected, and we're not in the middle of a head shake
            random_motion()
