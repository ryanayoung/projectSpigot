"""A simple test example. This example also works with glidepoint_lite.py"""
import time
import struct
import board
from digitalio import DigitalInOut
import circuitpython_cirque_pinnacle.glidepoint as Pinnacle

dr_pin = DigitalInOut(board.D2)
# NOTE The dr_pin is an optional keyword argument to the
# constructor when using Absolute or Relative modes

# if using a trackpad configured for SPI
spi = board.SPI()
ss_pin = DigitalInOut(board.D7)
tpad = Pinnacle.PinnacleTouchSPI(spi, ss_pin)
# if using a trackpad configured for I2C
# i2c = board.I2C()
# tpad = Pinnacle.PinnacleTouchI2C(i2c)

tpad.data_mode = Pinnacle.RELATIVE # ensure Absolute mode is enabled
tpad.absolute_mode_config(z_idle_count=1) # limit idle packet count to 1
tpad.set_adc_gain(1)
tpad.tune_edge_sensitivity()

print("test")

def print_data(timeout=10):
    """Print available data reports from the Pinnacle touch controller
    until there's no input for a period of ``timeout`` seconds."""
    if tpad.data_mode == Pinnacle.RELATIVE:
        print("using Relative mode")
    elif tpad.data_mode == Pinnacle.ABSOLUTE:
        print("using Absolute mode")
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if dr_pin.value: # is there new data?
            data = tpad.report(only_new=False)
            # Because we did not specify the dr_pin when instantiating the tpad variable,
            #   only_new=False skips the extra SPI or I2C transaction to check the
            #   SW_DR flag in the STATUS register which is reflected on the dr_pin

            if tpad.data_mode == Pinnacle.ABSOLUTE and data[3]:
                # NOTE ``and data[3]`` means only when Z-axis is > 0
                # specification sheet recommends clamping absolute position data of
                # X & Y axis for reliability
                data[1] = max(128, min(1920, data[1]))  # X-axis
                data[2] = max(64, min(1472, data[2]))  # Y-axis
            elif tpad.data_mode == Pinnacle.RELATIVE:
                # convert 2's compliment form into natural numbers
                data = struct.unpack('Bbbb', data)
            print(data)
            start = time.monotonic()

print_data()