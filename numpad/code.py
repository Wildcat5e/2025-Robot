"""
Example of a MacroPad as a game controller/joystick with multiple buttons.
Each key on the MacroPad maps to a game controller button. The mapping
is simply the game controller button number equals the MacroPad key number
plus one.

As a debugging tool, the MacroPad key number and state is printed on state change.
"""

import board
import usb_hid
from adafruit_macropad import MacroPad
from hid_gamepad import Gamepad

macropad = MacroPad()
gamepad = Gamepad(usb_hid.devices)

RED = 0xff0000
GREEN = 0x00ff00
BLUE = 0x0000ff
YELLOW = 0xffff00
PURPLE = 0xff00ff
BLACK = 0x000000

"""
Red for elevator jog up and outtake jog forward,
Green for elevator levels,
Blue for elevator jog down and outtake jog backwards, and
black for nothing.
"""
keys = (
    (0, BLACK, ""),
    (1, BLACK, ""),
    (2, BLACK, ""),
    (3, BLACK, ""),
    (4, GREEN, "elevator level 3"),
    (5, BLACK, ""),
    (6, RED, "elevator jog up"),
    (7, GREEN, "elevator level 2"),
    (8, RED, "elevator jog down"),
    (9, BLUE, "outtake jog out"),
    (10, GREEN, "elevator level 1"),
    (11, BLUE, "outtake jog in")
)

while True:
    try:
        if key_event := macropad.keys.events.get():

            if key_event.pressed:
                gamepad.press_buttons(key_event.key_number + 1)
                color = keys[key_event.key_number][1]
                message = keys[key_event.key_number][2]
                macropad.pixels.fill(BLACK)
                macropad.pixels[key_event.key_number] = color
                print(message)
            elif key_event.released:
                gamepad.release_buttons(key_event.key_number + 1)
    except Exception as e:
        print(e)