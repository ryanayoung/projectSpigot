""" a test example using SPI to read ADC measurements from the Pinnacle touch
controller in "AnyMeas" mode. This example does NOT work with
glidepoint_lite.py"""
import time
import struct
import board
from digitalio import DigitalInOut
import circuitpython_cirque_pinnacle.glidepoint as Pinnacle

dr_pin = DigitalInOut(board.D2)
# NOTE The dr_pin is a required keyword argument to the
# constructor when using AnyMeas mode

# if using a trackpad configured for SPI
spi = board.SPI()
ss_pin = DigitalInOut(board.D7)
tpad = Pinnacle.PinnacleTouchSPI(spi, ss_pin, dr_pin=dr_pin)
# if using a trackpad configured for I2C
# i2c = board.I2C()
# tpad = Pinnacle.PinnacleTouchI2C(i2c, dr_pin=dr_pin)

# if dr_pin was not specified upon instantiation.
# this command will raise an AttributeError exception
tpad.data_mode = Pinnacle.ANYMEAS

# setup toggle and polarity bits for measuring with PNP gate muxing
class MeasVector:
    """A blueprint matrix used to manipulate the measurements' vector"""

    def __init__(self, toggle, polarity):
        self.toggle = toggle
        self.polarity = polarity


vectors = []
# This toggles Y0 only and toggles it positively
vectors.append(MeasVector(0x00010000, 0x00010000))
# This toggles Y0 only and toggles it negatively
vectors.append(MeasVector(0x00010000, 0x00000000))
# This toggles X0 only and toggles it positively
vectors.append(MeasVector(0x00000001, 0x00000000))
# This toggles X16 only and toggles it positively
vectors.append(MeasVector(0x00008000, 0x00000000))
# This toggles Y0-Y7 negative and X0-X7 positive
vectors.append(MeasVector(0x00FF00FF, 0x000000FF))

idle_vectors = [0] * len(vectors)


def compensate(count=5):
    """take ``count`` measurements, then average them together  """
    for i, vector in enumerate(vectors):
        idle_vectors[i] = 0
        for _ in range(count):
            result = struct.unpack(
                "h",
                tpad.measure_adc(vector.toggle, vector.polarity)
            )[0]
            idle_vectors[i] += result
        idle_vectors[i] /= count
        print("compensation {}: {}".format(i, idle_vectors[i]))


def take_measurements(timeout=10):
    """read ``len(vectors)`` number of measurements and print results for
    ``timeout`` number of seconds."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        for i, vector in enumerate(vectors):
            result = struct.unpack(
                "h",
                tpad.measure_adc(vector.toggle, vector.polarity)
            )[0]
            print("vector{}: {}".format(i, result - idle_vectors[i]), end="\t")
        print()
