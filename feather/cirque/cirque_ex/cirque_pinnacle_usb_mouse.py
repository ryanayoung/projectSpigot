""" This example uses CircuitPython's built-in `usb_hid` API
to emulate a mouse with the Cirque circle trackpad. This example
also works with glidepoint_lite.py"""
import time
import board
import struct
from digitalio import DigitalInOut
import usb_hid
import circuitpython_cirque_pinnacle.glidepoint as Pinnacle

dr_pin = DigitalInOut(board.D2)
# NOTE Specifying the optional keyword argument ``dr_pin`` to the
# constructor expedites ``report()`` when using Absolute or Relative modes

# if using a trackpad configured for SPI
spi = board.SPI()
ss_pin = DigitalInOut(board.D7)
tpad = Pinnacle.PinnacleTouchSPI(spi, ss_pin, dr_pin=dr_pin)
# if using a trackpad configured for I2C
# i2c = board.I2C()
# tpad = Pinnacle.PinnacleTouchI2C(i2c, dr_pin=dr_pin)

tpad.data_mode = Pinnacle.RELATIVE  # ensure mouse mode is enabled
tpad.set_adc_gain(0)
tpad.tune_edge_sensitivity()

mouse = None
for dev in usb_hid.devices:
    # be sure we're grabbing the mouse singleton
    if dev.usage == 2 and dev.usage_page == 1:
        mouse = dev
# mouse.send_report() takes a 4 byte buffer in which
#   byte0 = buttons in which
#       bit5 = back, bit4 = forward, bit2 = middle, bit1 = right, bit0 = left
#   byte1 = delta x-axis
#   byte2 = delta y-axis
#   byte3 = delta scroll wheel

def move(timeout=10):
    """Send mouse X & Y reported data from the Pinnacle touch controller
    until there's no input for a period of ``timeout`` seconds."""
    if mouse is None:
        raise OSError("mouse HID device not available.")
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        data = tpad.report()  # only returns fresh data (if any)
        if data:  # is there fresh data?
            data[1] = 255 - data[1]
            mouse.send_report(data)  # no scrolling or backward/forward
            start = time.monotonic()
    mouse.send_report(b'\x00' * 4)  # release buttons (just in case)

move()