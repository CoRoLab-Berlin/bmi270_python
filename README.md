# BMI270 I2C Python Implementation - Version: 0.3

WIP BMI270 I2C Python implementation.

## Installation
The package is [available on pypi.org](https://pypi.org/project/bmi270).

You can install this package using this command:
`pip3 install bmi270`

## Usage
Wire the breakout board with these lines : GND, 3V3, SDA, SCL

Make sure that the device is available at `0x68` or `0x69` i2c address by running this command:

`i2cdetect -y 1`

## Tested with:
- Ubuntu 22.04.2 LTS
- Raspbian Buster (32 Bit)

## Dependencies

#### smbus2
**This library requires [smbus2](https://github.com/kplindegaard/smbus2)** (auto installed)

Manual installation:
`pip3 install smbus2`

**H5py** (for data plotting)
Ubuntu:
`pip3 install h5py`

Raspbian:
`sudo apt-get install python3-h5py`

## Credits & Related links

- [serioeseGmbH/BMI160](https://github.com/serioeseGmbH/BMI160)
- [lefuturiste](https://github.com/lefuturiste/BMI160-i2c)
- [BMI270 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
- [smbus2 docs](https://smbus2.readthedocs.io/en/latest/)