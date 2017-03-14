#! /usr/bin/env python3

from distutils.core import setup


def readme():
    with open('README.md') as f:
        return f.read()

setup(
        name="PyADXL345",
        version="0.1",
        author="giuaig",
        author_email="gibiz@gmx.com",
        description="A python library for accessing ADXL345 accelerometer",
        long_description=readme(),
        license="GNU GPLv3",
        packages=['PyADXL345']
        )
