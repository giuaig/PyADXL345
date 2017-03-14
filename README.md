# PyADXL345
A Python library for accessing ADXL345 accelerometer using i2C on the Raspberry Pi

## Example
Assuming that the address of ADXL345 accelerometer is 0x53, you can access data with:
```
>>> from PyADXL345 import PyADXL345
>>> adxl345 = PyADXL345.ADXL345(debug=False)
>>> adxl345.enableAll()
```

## Dependencies
* sudo apt-get install python3-smbus

## Installation
```
sudo python3 setup.py install
```

## Features
* Read data from axes RAW, in G, in m/s^2, low pass filtered, rounded
* Activity/Inactivity/Single Tap/Double Tap/Free Falling 
* Get Activity and Tap axis input source
* Pitch & Roll

## License
See ./LICENSE

## Changelog
See ./CHANGELOG

## Issues & Bugs
Please report any issues or bugs here:
https://github.com/
