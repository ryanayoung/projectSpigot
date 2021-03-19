# SPDX-FileCopyrightText: 2018 Dan Halbert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_hid.gamepad.Gamepad`
====================================================

* Author(s): Dan Halbert
"""

import struct
import time

from . import find_device


class Gamepad:
    """Emulate a generic gamepad controller with 16 buttons,
    numbered 1-16, and two joysticks, one controlling
    ``x` and ``y`` values, and the other controlling ``z`` and
    ``r_z`` (z rotation or ``Rz``) values.

    The joystick values could be interpreted
    differently by the receiving program: those are just the names used here.
    The joystick values are in the range -127 to 127."""

    def __init__(self, devices):
        """Create a Gamepad object that will send USB gamepad HID reports.

        Devices can be a list of devices that includes a gamepad device or a gamepad device
        itself. A device is any object that implements ``send_report()``, ``usage_page`` and
        ``usage``.
        """
        self._gamepad_device = find_device(devices, usage_page=0x1, usage=0x05)

        # Reuse this bytearray to send mouse reports.
        # Typically controllers start numbering buttons at 1 rather than 0.
        # report[0] buttons 1-8 (LSB is button 1)
        # report[1] buttons 9-16
        # report[2] buttons 17-24
        # report[3] buttons 25-32

        # report[10] joystick 0 x: -127 to 127
        # report[11] joystick 0 y: -127 to 127
        # report[12] trigger  L z: -127 to 127
        # report[13] joystick 1 x: 
        # report[14] joystick 1 y:
        # report[15] trigger  R rz: -127 to 127


        self._report = bytearray(13)

        # Remember the last report as well, so we can avoid sending
        # duplicate reports.
        self._last_report = bytearray(13)

        # Store settings separately before putting into report. Saves code
        # especially for buttons.
        self._buttons_state = 0
        self._joy_x = 0
        self._joy_y = 0
        self._joy_r_x = 0
        self._joy_r_y = 0
        self._joy_z = 0
        self._joy_r_z = 0
        self._hat1 = 0
        # If not, wait a bit and try once more.
        try:
            self.reset_all()
        except OSError:
            time.sleep(1)
            self.reset_all()

    def press_buttons(self, *buttons):
        """Press and hold the given buttons. """
        for button in buttons:
            self._buttons_state |= 1 << self._validate_button_number(button) - 1
        self._send()
    
    def press_hat(self, *directions):
        """Press and hold the given buttons. """
        for directions in directions:
            self._hat1 |= 1 << directions - 1
        self._send()
    
    def release_buttons(self, *buttons):
        """Release the given buttons. """
        for button in buttons:
            self._buttons_state &= ~(1 << self._validate_button_number(button) - 1)
        self._send()

    def release_hat(self, *directions):
        """Release the given buttons. """
        for directions in directions:
            self._hat1 &= ~(1 << directions - 1)
        self._send()

    def release_all_buttons(self):
        """Release all the buttons."""

        self._buttons_state = 0
        self._send()
    
    def release_all_hat(self):
        """Release all the buttons."""

        self._hat1 = 0
        self._send()

    def click_buttons(self, *buttons):
        """Press and release the given buttons."""
        self.press_buttons(*buttons)
        self.release_buttons(*buttons)
    
    def click_hat(self, *directions):
        """Press and release the given buttons."""
        self.press_hat(*directions)
        self.release_hat(*directions)

    def move_joysticks(self, x=None, y=None, r_x=None, r_y=None, z=None, r_z=None):
        """Set and send the given joystick values.
        The joysticks will remain set with the given values until changed

        One joystick provides ``x`` and ``y`` values,
        and the other provides ``z`` and ``r_z`` (z rotation).
        Any values left as ``None`` will not be changed.

        All values must be in the range -127 to 127 inclusive.

        Examples::

            # Change x and y values only.
            gp.move_joysticks(x=100, y=-50)

            # Reset all joystick values to center position.
            gp.move_joysticks(0, 0, 0, 0)
        """
        if x is not None:
            self._joy_x = x
        if y is not None:
            self._joy_y = y
        if r_x is not None:
            self._joy_r_x = r_x
        if r_y is not None:
            self._joy_r_y = r_y
        if z is not None:
            self._joy_z = z
        if r_z is not None:
            self._joy_r_z = r_z
        self._send()

    def reset_all(self):
        """Release all buttons and set joysticks to zero."""
        self._buttons_state = 0
        self._joy_x = 0
        self._joy_y = 0
        self._joy_r_x = 0
        self._joy_r_y = 0
        self._joy_z = 0
        self._joy_r_z = 0
        self._hat1 = 0
        self._send(always=True)

    def _send(self, always=False):
        """Send a report with all the existing settings.
        If ``always`` is ``False`` (the default), send only if there have been changes.
        """
        struct.pack_into(
            "<Hhhhhbbb",
            self._report,
            0,
            self._buttons_state,
            self._joy_x,
            self._joy_y,
            self._joy_r_x,
            self._joy_r_y,
            self._joy_z,
            self._joy_r_z,
            self._hat1,
        )

        if always or self._last_report != self._report:
            self._gamepad_device.send_report(self._report)
            # Remember what we sent, without allocating new storage.
            self._last_report[:] = self._report

    @staticmethod
    def _validate_button_number(button):
        if not 1 <= button <= 16:
            raise ValueError("Button number must in range 1 to 32")
        return button

    @staticmethod
    def _validate_joystick_value(value):
        if not -127 <= value <= 127:
            raise ValueError("Joystick value must be in range -127 to 127")
        return value
