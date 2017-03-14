#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#
# apt-get install python3-smbus
#
# Author:
# License:
# Datasheet: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.PDF
#
try:
    import smbus
except ImportError:
    raise RuntimeError('Missing module! Install it with apt-get install python3-smbus')
try:
    from PyADXL345 import PyADXL345
    import math
    import time
    import sys
except ImportError:
    raise RuntimeError('You need to install required modules first!')

INTERVAL = 0.5
prevTime = 0

if __name__ == "__main__":
    adxl345 = PyADXL345.ADXL345(debug=False)
    adxl345.enableAll()

    try:
        while True:
            now = time.time()

            if (now - prevTime >= INTERVAL):
                prevTime = now

                status = adxl345.getIntStatus()
                act_tap_axes = adxl345.printActTapAxes()

                if adxl345.isInactivity(status):
                    print("--> Inactivity")
                if adxl345.isActivity(status):
                    sourceAxes = adxl345.printActAxes(act_tap_axes)
                    print("==> Activity (" + sourceAxes + ")")
                if adxl345.isDoubleTap(status):
                    sourceAxes = adxl345.printTapAxes(act_tap_axes)
                    print("Double Tap (" + sourceAxes  + ")")
                if adxl345.isSingleTap(status):
                    sourceAxes = adxl345.printTapAxes(act_tap_axes)
                    print("Single Tap (" + sourceAxes  + ")")
                if adxl345.isFreeFall(status):
                    print("---Falling!")

                ## Read X/Y/Z Axes Values
                axes = adxl345.readRaw()
                faxes = adxl345.readFiltered(axes)
                roll = adxl345.roll(faxes, rounded=True)
                pitch = adxl345.pitch(faxes, rounded=True)
                print("RAW: \tx=" , (axes['x']), "\ty=" , (axes['y']), "\tz=" , (axes['z']), "\tRoll=", roll, "\tPitch=", pitch)
                axesG = adxl345.readInG(axes, rounded=True)
                print("G: \tx=" , (axesG['x']), "\ty=" , (axesG['y']), "\tz=" , (axesG['z']))
                axesMS = adxl345.readInMS(axesG, rounded=True)
                print("M/S^2: \tx=" , (axesMS['x']), "\ty=" , (axesMS['y']), "\tz=" , (axesMS['z']))

            #time.sleep(0.5)

    except KeyboardInterrupt:
        sys.exit()

##EOF
