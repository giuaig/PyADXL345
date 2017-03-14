#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#
# '''Library for ADXL345 accelerometer on a RaspberryPi'''
#
# apt-get install python3-smbus
#
# Author: giuaig
# License: GPLv2
# Version: 0.1
# Datasheet: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.PDF
#
# TODO:
# -Offset/Calibration/Self_Test
# -FIFO stuff

try:
    import smbus
except ImportError:
    raise RuntimeError('Missing module! Install it with apt-get install python3-smbus')
import math

# Select the correct i2c bus for this revision of Raspberry Pi
revision = ([l[12:-1] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]
bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)

# Register Map (DEC)
DEVID               = 0x00 # (0) R Device ID
THRESH_TAP          = 0x1D # (29) R/W Tap threshold
OFSX                = 0x1E # (30) R/W X-axis offset
OFSY                = 0x1F # (31) R/W Y-axis offset
OFSZ                = 0x20 # (32) R/W Z-axis offset
DUR                 = 0x21 # (33) R/W Tap duration
LATENT              = 0x22 # (34) R/W Tap latency
WINDOW              = 0x23 # (35) R/W Tap window
THRESH_ACT          = 0x24 # (36) R/W Activity threshold
THRESH_INACT        = 0x25 # (37) R/W Inactivity threshold
TIME_INACT          = 0x26 # (38) R/W Inactivity time
ACT_INACT_CTL       = 0x27 # (39) R/W Axis enable control for activity and inactivity detection
THRESH_FF           = 0x28 # (40) R/W Free-fall threshold
TIME_FF             = 0x29 # (41) R/W Free-fall time
TAP_AXES            = 0x2A # (42) R/W Axis Control for Single Tap/Double Tap
ACT_TAP_STATUS      = 0x2B # (43) R Source of Single Tap/Double Tap
BW_RATE             = 0x2C # (44) R/W Data rate and power mode control
POWER_CTL           = 0x2D # (45) R/W Power-saving features control
INT_ENABLE          = 0x2E # (46) R/W Interrupt enable control
INT_MAP             = 0x2F # (47) R/W Interrupt mapping control
INT_SOURCE          = 0x30 # (48) R Source of Interrupts
DATA_FORMAT         = 0x31 # (49) R/W Data Format Control
DATAX0              = 0x32 # (50) R X-axis data 0 (6 bytes for X/Y/Z)
DATAX1              = 0x33 # (51) R X-Axis Data 1
DATAY0              = 0x34 # (52) R Y-Axis Data 0
DATAY1              = 0x35 # (53) R Y-Axis Data 1
DATAZ0              = 0x36 # (54) R Z-Axis Data 0
DATAZ1              = 0x37 # (55) R Z-Axis Data 1
FIFO_CTL            = 0x38 # (56) R/W FIFO control
FIFO_STATUS         = 0x39 # (57) R FIFO status

# BW_RATE Rate Parameters (0x2C)
DATARATE_0_10_HZ    = 0x00
DATARATE_0_20_HZ    = 0x01
DATARATE_0_39_HZ    = 0x02
DATARATE_0_78_HZ    = 0x03
DATARATE_1_56_HZ    = 0x04
DATARATE_3_13_HZ    = 0x05
DATARATE_6_25HZ     = 0x06
DATARATE_12_5_HZ    = 0x07
DATARATE_25_HZ      = 0x08
DATARATE_50_HZ      = 0x09
DATARATE_100_HZ     = 0x0A # (default)
DATARATE_200_HZ     = 0x0B
DATARATE_400_HZ     = 0x0C
DATARATE_800_HZ     = 0x0D
DATARATE_1600_HZ    = 0x0E
DATARATE_3200_HZ    = 0x0F

# DATA_FORMAT Parameters (0x31)
SELF_TEST_ON   = 0x80 # 0b10000000
SELF_TEST_OFF  = 0x00 # 0b00000000
SPI_ON         = 0x40 # 0b01000000
SPI_OFF        = 0x00 # 0b00000000
INT_INVERT_ON  = 0x20 # 0b00100000
INT_INVERT_OFF = 0x00 # 0b00000000
FULL_RES_ON    = 0x08 # 0b00001000
FULL_RES_OFF   = 0x00 # 0b00000000
JUSTIFY_ON     = 0x04 # 0b00000100
JUSTIFY_OFF    = 0x00 # 0b00000000
RANGE_2G       = 0x00 # 0b00000000 +/-  2g (default)
RANGE_4G       = 0x01 # 0b00000001 +/-  4g
RANGE_8G       = 0x02 # 0b00000010 +/-  8g
RANGE_16G      = 0x03 # 0b00000011 +/- 16g

# ACT_TAP_STATUS Parameters (0x2B)
ACT_AXES_X      = 0x40 # 0b01000000
ACT_AXES_Y      = 0x20 # 0b00100000
ACT_AXES_Z      = 0x10 # 0b00010000
TAP_AXES_X      = 0x04 # 0b00000100
TAP_AXES_Y      = 0x02 # 0b00000010
TAP_AXES_Z      = 0x01 # 0b00000001

# INT_ENABLE (0x2E) / INT_MAP (0x2F) / INT_SOURCE (0x30)
INT_SINGLE_TAP  = 0x40 # 0b01000000
INT_DOUBLE_TAP  = 0x20 # 0b00100000
INT_ACT         = 0x10 # 0b00010000 activity
INT_INACT       = 0x08 # 0b00001000 inactivity
INT_FF          = 0x04 # 0b00000100 free_fall

# POWER_CTL Parameters (0x2D)
LINK_ON         = 0x20 # 0b00100000
LINK_OFF        = 0x00 # 0b00000000
AUTO_SLEEP_ON   = 0x10 # 0b00010000
AUTO_SLEEP_OFF  = 0x00 # 0b00000000
MEASURE_ON      = 0x08 # 0b00001000
MEASURE_OFF     = 0x00 # 0b00000000
SLEEP_ON        = 0x04 # 0b00000100
SLEEP_OFF       = 0x00 # 0b00000000
WAKEUP_1HZ      = 0x03 # 0b00000011
WAKEUP_2HZ      = 0x02 # 0b00000010
WAKEUP_4HZ      = 0x01 # 0b00000001
WAKEUP_8HZ      = 0x00 # 0b00000000

EARTH_GRAVITY_MS2 = 9.80665
SCALE_MULTIPLIER = 0.0039
ALPHA = float(0.5)

STD_ADDRESS     = 0x1D
ALT_ADDRESS     = 0x53 # Device Address (with SDO tied to ground)

class ADXL345:
    """ADXL345 Accelerometer"""
    def __init__(self, address=ALT_ADDRESS, debug=False):
        self.debug = debug
        self.address = address

        self.getIntStatus() #we need to clear INT_SOURCE by reading it (for false positives)
        self.config() #disable everything on start (Single/Double tap, Activity/Inactivity, FreeFall)

        print("...Init End...")


    ## Set values in DATA_FORMAT (0x31) register
    ## SELF_TEST / SPI / INT_INVERT / 0 / FULL_RES / JUSTIFY / RANGE
    def setDataFormat(self, selftest=SELF_TEST_OFF, spi=SPI_OFF, invert=INT_INVERT_OFF, fullres=FULL_RES_ON, justify=JUSTIFY_OFF, rangeG=RANGE_2G):
        conf = selftest | spi | invert | fullres | justify | rangeG
        bus.write_byte_data(self.address, DATA_FORMAT, conf) #0x31

    ## Read DATA_FORMAT Register (0x31)
    ## SELF_TEST / SPI / INT_INVERT / 0 / FULL_RES / JUSTIFY / RANGE
    def getDataFormat(self):
        dataFormat = bus.read_byte_data(self.address, DATA_FORMAT) #0x31
        if self.debug == True:
            print("DATA_FORMAT (0x31): " + format(dataFormat, '#010b'))
        return dataFormat

    ## Set values in POWER_CTL (0x2D) register
    ## 0 / 0 / LINK / AUTO_SLEEP / MEASURE / SLEEP / WAKEUP
    def setPowerCtl(self, link=LINK_ON, auto_sleep=AUTO_SLEEP_ON, measure=MEASURE_ON, sleep=SLEEP_OFF, wakeup=WAKEUP_8HZ):
        power = link | auto_sleep | measure | sleep | wakeup
        bus.write_byte_data(self.address, POWER_CTL, power) #0x2D

    ## Read values in POWER_CTL (0x2D) register
    ## 0 / 0 / LINK / AUTO_SLEEP / MEASURE / SLEEP / WAKEUP
    def getPowerCtl(self):
        powerCtl = bus.read_byte_data(self.address, POWER_CTL) #0x2D
        if self.debug == True:
            print("POWER_CTL (0x2D): " + format(powerCtl, '#010b'))
        return powerCtl

    ## Send all interrupts to ADXL345 INT1 pin or INT2 pin
    ## 0 send to INT1 physical pin -- 1 send to INT2 physical pin
    ## Default: send everything to INT1
    def setIntMap(self, single_tap=0, double_tap=0, activity=0, inactivity=0, freefall=0):
        if single_tap == 1:
            single_tap = INT_SINGLE_TAP
        elif single_tap == 0:
            single_tap = 0x00
        if double_tap == 1:
            double_tap = INT_DOUBLE_TAP
        elif double_tap == 0:
            double_tap = 0x00
        if activity == 1:
            activity = INT_ACT
        elif activity == 0:
            activity = 0x00
        if inactivity == 1:
            inactivity = INT_INACT
        elif inactivity == 0:
            inactivity = 0x00
        if freefall == 1:
            freefall = INT_FF
        elif freefall == 0:
            freefall = 0x00
        intMap = single_tap | double_tap | activity | inactivity | freefall
        bus.write_byte_data(self.address, INT_MAP, intMap) #0x2F

    ## Read values from INT_MAP (0x2F)
    def getIntMap(self):
        intMap = bus.read_byte_data(self.address, INT_MAP) #0x2F
        if self.debug == True:
            print("INT_MAP (0x2F): " + format(intMap, '#010b'))
        return intMap

    ## Set Default INT_ENABLE register configuration
    ##
    def config(self, bits=0x00): #0b00000000
        bus.write_byte_data(self.address, INT_ENABLE, bits) #0x2E - disable single/double tap, activity, inactivity, free fall

    ## Read fixed Device ID from DEVID Register (0x00)
    def getDevid(self):
        devid = bus.read_byte_data(self.address, DEVID)
        if self.debug == True:
            print("DEVID (0x00): " + format(devid, '#010b'))
        return devid

    ## Read from INT_ENABLE Register (0x2E)
    def getIntEnable(self):
        intEnable =  bus.read_byte_data(self.address, INT_ENABLE)
        if self.debug == True:
            print("INT_ENABLE (0x2E): " + format(intEnable, '#010b'))
        return intEnable

    ## Read Interrupt status from INT_SOURCE Register (0x30)
    def getIntStatus(self):
        intStatus = bus.read_byte_data(self.address, INT_SOURCE) #0x30
        if self.debug == True:
            print("INT_SOURCE (0x30): " + format(intStatus, '#010b'))
        return intStatus

    ## Get current Power Mode
    def getPowerMode(self):
        powerCTL = bus.read_byte_data(self.address, POWER_CTL) #0x2D
        if self.debug == True:
            print("POWER_CTL (0x2D): " + format(powerCTL, '#010b'))
        return powerCTL

    ## Read current values in TAP_AXES (0x2A)
    def getTapAxes(self):
        tapAxes = bus.read_byte_data(self.address, TAP_AXES) #0x2A
        if self.debug == True:
            print("TAP_AXES (0x2A): " + format(tapAxes, '#010b'))
        return tapAxes

    ## Read current values in ACT_INACT_CTL (0x27)
    def getActInactCtl(self):
        actInactCtl = bus.read_byte_data(self.address, ACT_INACT_CTL) #0x27
        if self.debug == True:
            print("ACT_INACT_CTL (0x27): " + format(actInactCtl, '#010b'))
        return actInactCtl

    ## Activity detection configuration
    def confActivity(self, thresh_act=7):
        bus.write_byte_data(self.address, THRESH_ACT, thresh_act) # 62.5mg/LSB

        int_act = INT_ACT #0b00010000
        intEnable = self.getIntEnable() #read current INT_ENABLE value before change bits
        #bus.write_byte_data(self.address, INT_ENABLE, int_act) #0x2E
        bus.write_byte_data(self.address, INT_ENABLE, intEnable | int_act) #0x2E

    ## Inactivity detection configuration
    def confInactivity(self, time_inact=10, thresh_inact=3):
        bus.write_byte_data(self.address, TIME_INACT, time_inact) # 1sec/LSB
        bus.write_byte_data(self.address, THRESH_INACT, thresh_inact) # 62.5mg/LSB

        int_inact = INT_INACT #0b00001000
        intEnable = self.getIntEnable() #read current INT_ENABLE value before change bits
        #bus.write_byte_data(self.address, INT_ENABLE, int_inact) #0x2E
        bus.write_byte_data(self.address, INT_ENABLE, intEnable | int_inact) #0x2E

    ## Activity and Inactivity control
    def enableActInact(self, act_inact_ctl=0xFF, link_ctl=0x30):
        bus.write_byte_data(self.address, ACT_INACT_CTL, 0b11111111) #0x27 - Set defaults
        int_actInactCtl = self.getActInactCtl()
        ##bus.write_byte_data(self.address, ACT_INACT_CTL, act_inact_ctl) #0x27
        #bus.write_byte_data(self.address, ACT_INACT_CTL, 0b11111111) #0x27
        bus.write_byte_data(self.address, ACT_INACT_CTL, int_actInactCtl & 0b11111111) #0x27

        # Set the Link bit to 1 so that Activity and Inactivity aren't concurrent
        # Set Auto Sleep bit to 1 so that the device automatically goes sleeping when inactivity detected
        ##bus.write_byte_data(self.address, POWER_CTL, link_ctl) #0x2D
        powerCTL = self.getPowerMode() #0x2D
        bus.write_byte_data(self.address, POWER_CTL, powerCTL | 0b00110000) #0x2D


    ## Check if there is Activity
    def isActivity(self, value): #value from getIntStatus() 0x30
        if((value & INT_ACT) == INT_ACT): #0b00010000
            return True
        else:
            #print("false")
            return False

    ## Check if there is Inactivity
    def isInactivity(self, value):
        if((value & INT_INACT) == INT_INACT): #0b00001000
            return True
        else:
            return False

    ## Enable Single Tap
    ## Default Tap detecting on axes X/Y/Z
    def enableTap(self, thresh_tap=40, dur=32, latent=80, window=240, x=True, y=True, z=True):
        bus.write_byte_data(self.address, THRESH_TAP, thresh_tap) # 62.5mg/LSB
        bus.write_byte_data(self.address, DUR, dur)               # 625us/LSB
        bus.write_byte_data(self.address, LATENT, latent)         # 1.25ms/LSB
        bus.write_byte_data(self.address, WINDOW, window)         # 1.25ms/LSB

        int_tap = INT_SINGLE_TAP | INT_DOUBLE_TAP
        intEnable = self.getIntEnable() #read current INT_ENABLE value before change bits
        bus.write_byte_data(self.address, INT_ENABLE, intEnable | int_tap)  #0x2E Interrupts Tap Enable

        bus.write_byte_data(self.address, TAP_AXES, 0b00000111) #initialize for proper axis bit change
        tapAxes = self.getTapAxes() #get current Axes value
        #TAP_AXES_X == 0b00000100
        if x == True:
            bus.write_byte_data(self.address, TAP_AXES, tapAxes | TAP_AXES_X) # 0x2A - Enable X axis Tap
        elif x == False:
            bus.write_byte_data(self.address, TAP_AXES, tapAxes ^ TAP_AXES_X) # 0x2A - Disable X axis Tap
        tapAxes = self.getTapAxes() #get current Axes value
        #TAP_AXES_Y == 0b00000010
        if y == True:
            bus.write_byte_data(self.address, TAP_AXES, tapAxes | TAP_AXES_Y) # 0x2A - Enable Y axis Tap
        elif y == False:
            bus.write_byte_data(self.address, TAP_AXES, tapAxes ^ TAP_AXES_Y) # 0x2A - Disable Y axis Tap
        tapAxes = self.getTapAxes() #get current Axes value
        #TAP_AXES_Z == 0b00000001
        if z == True:
            bus.write_byte_data(self.address, TAP_AXES, tapAxes | TAP_AXES_Z) # 0x2A - Enable Z axis Tap
        elif z == False:
            bus.write_byte_data(self.address, TAP_AXES, tapAxes ^ TAP_AXES_Z) # 0x2A - Disable Z axis Tap

    ## Check if Single Tap
    def isSingleTap(self, value):
        if((value & INT_SINGLE_TAP) == INT_SINGLE_TAP):
            return True
        else:
            return False

    ## Check if Double Tap
    def isDoubleTap(self, value):
        if((value & INT_DOUBLE_TAP) == INT_DOUBLE_TAP):
            return True
        else:
            return False

    ## Free Fall Detection
    def enableFreeFall(self, thresh_ff=6, time_ff=20):
        bus.write_byte_data(self.address, THRESH_FF, thresh_ff) # 300mg to 600mg recommended (0x05 to 0x09)
        bus.write_byte_data(self.address, TIME_FF, time_ff) # 100ms to 350ms recommended (0x14 to 0x46)

        int_ff = INT_FF #0b00000100
        intEnable = self.getIntEnable() #read current INT_ENABLE value before change bits
        bus.write_byte_data(self.address, INT_ENABLE, intEnable | int_ff) #0x2E

    ## Check if Free Fall
    def isFreeFall(self, value):
        if((value & INT_FF) == INT_FF): #0b00000100
            return True
        else:
            return False

    ## Read from ACT_TAP_STATUS Register (0x2B)
    def getActTapStatus(self):
        intActTapStatus = bus.read_byte_data(self.address, ACT_TAP_STATUS) #0x2B
        if self.debug == True:
            print("ACT_TAP_STATUS (0x2B): " + format(intActTapStatus, '#010b'))
        return intActTapStatus

    ## Print Tap Axes
    def printActTapAxes(self):
        value = self.getActTapStatus()
        act_tap = [False, False, False, False, False, False] #[ActX, ActY, ActZ, TapX, TapY, TapZ]
        if((value & ACT_AXES_X) == ACT_AXES_X): #0b01000000
            act_tap[0] = True
            #print("Act(X)")
        if((value & ACT_AXES_Y) == ACT_AXES_Y): #0b00100000
            act_tap[1] = True
            #print("Act(Y)")
        if((value & ACT_AXES_Z) == ACT_AXES_Z): #0b00010000
            act_tap[2] = True
            #print("Act(Z)")
        if((value & TAP_AXES_X) == TAP_AXES_X): #0b00000100
            act_tap[3] = True
            #print("Tap(X)")
        if((value & TAP_AXES_Y) == TAP_AXES_Y): #0b00000010
            act_tap[4] = True
            #print("Tap(Y)")
        if((value & TAP_AXES_Z) == TAP_AXES_Z): #0b00000001
            act_tap[5] = True
            #print("Tap(Z)")
        return act_tap

    ## Print only Activity axes source
    def printActAxes(self, act_tap_axes):
        actString = ""
        if act_tap_axes[0] == True:
            #print("X", end='')
            actString = actString + "X"
        if act_tap_axes[1] == True:
            #print("Y", end='')
            actString = actString + "Y"
        if act_tap_axes[2] == True:
            #print("Z", end='')
            actString = actString + "Z"
        return actString

    ## Print only Tap axes source
    def printTapAxes(self, act_tap_axes):
        tapString = ""
        if act_tap_axes[3] == True:
            #print("X", end='')
            tapString = tapString + "X"
        if act_tap_axes[4] == True:
            #print("Y", end='')
            tapString = tapString + "Y"
        if act_tap_axes[5] == True:
            #print("Z", end='')
            tapString = tapString + "Z"
        return tapString

    ## Read Raw XYZ Axes data
    def readRaw(self):
        data = bus.read_i2c_block_data(self.address, DATAX0, 6) #read 6 bytes, so XYZ

        x = self.dataConv(data[0], data[1])
        y = self.dataConv(data[2], data[3])
        z = self.dataConv(data[4], data[5])

        return {"x":x, "y":y, "z":z}

    ## Return XYZ Axes Raw multiplied by scale factor to get acceleration in g
    def readInG(self, values, rounded=False):
        x = values['x'] * SCALE_MULTIPLIER
        y = values['y'] * SCALE_MULTIPLIER
        z = values['z'] * SCALE_MULTIPLIER
        if rounded == False:
            return {"x":x, "y":y, "z":z}
        else:
            return {"x":round(x, 4), "y":round(y, 4), "z":round(z, 4)}

    ## Return XYZ Axes scaled multiplied by the earth gravity to get acceleration in m/s^2
    def readInMS(self, values, rounded=False):
        x = values['x'] * EARTH_GRAVITY_MS2
        y = values['y'] * EARTH_GRAVITY_MS2
        z = values['z'] * EARTH_GRAVITY_MS2
        if rounded == False:
            return {"x":x, "y":y, "z":z}
        else:
            return {"x":round(x, 4), "y":round(y, 4), "z":round(z, 4)}

    ## Get values from readRaw() and return filtered data to use in roll() and pitch() functions
    def readFiltered(self, values, alpha=ALPHA):
        x = float(values['x'])
        y = float(values['y'])
        z = float(values['z'])
        #Filtered axes
        fX = float(0)
        fY = float(0)
        fZ = float(0)
        #Low Pass Filter
        fX = x * alpha + (fX * (1.0 - alpha))
        fY = y * alpha + (fY * (1.0 - alpha))
        fZ = z * alpha + (fZ * (1.0 - alpha))
        return {"fX":fX, "fY":fY, "fZ":fZ}

    ## Get values from readFiltered() and return roll
    def roll(self, values, rounded=False):
        fY = values['fY']
        fZ = values['fZ']
        roll = (math.atan2(-fY, fZ)*180.0)/math.pi
        if rounded == False:
            return roll
        elif rounded == True:
            return round(roll, 4)

    ## Get values from readFiltered() and return pitch
    def pitch(self, values, rounded=False):
        fX = values['fX']
        fY = values['fY']
        fZ = values['fZ']
        pitch = (math.atan2(fX, math.sqrt(fY*fY + fZ*fZ))*180.0)/math.pi
        if rounded == False:
            return pitch
        elif rounded == True:
            return round(pitch, 4)

    ## Data Convert
    def dataConv(self, data1, data2):
        value = data1 | (data2 << 8)
        if(value & (1 << 16 - 1)):
            value -= (1<<16)
        return value

    ## Enable every interrupt in INT_ENABLE
    ## Single Tap, Double Tap, Activity, Inactivity, Free Fall
    def enableAll(self):
        self.setIntMap()
        self.setDataFormat()
        self.setPowerCtl() #0b00111000
        self.confActivity()
        self.confInactivity()
        self.enableActInact()
        self.enableFreeFall()
        self.enableTap()

##EOF
