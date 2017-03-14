# PyADXL345
A Python3 library for accessing ADXL345 accelerometer using i2C on the Raspberry Pi

## Example
Assuming that the address of ADXL345 accelerometer is 0x53, you can access data with:
```
>>> from PyADXL345 import PyADXL345
>>> adxl345 = PyADXL345.ADXL345()
>>> adxl345.enableAll()
>>> axes = adxl345.readRaw()
```

## Dependencies
```
sudo apt-get install python3-smbus
```

## Installation
```
sudo python3 setup.py install
```

## Features
* Read from axes in RAW data, in G, in m/s^2, low pass filtered, rounded
* Activity/Inactivity/Single Tap/Double Tap/Free Falling 
* Get Activity and Tap axis input source
* Pitch & Roll

## TODO
* Calibration
* FIFO

## License
See [LICENSE](https://github.com/giuaig/PyADXL345/blob/master/LICENSE)

## Changelog
See [CHANGELOG](https://github.com/giuaig/PyADXL345/blob/master/CHANGELOG)

## Issues & Bugs
Please report any issues or bugs here:
https://github.com/giuaig/PyADXL345
