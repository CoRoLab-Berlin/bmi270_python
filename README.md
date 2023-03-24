# BMI270 I2C Python Implementation

## Installation

`python3 -m pip install bmi270-python`

## Usage
Wire the breakout board with these lines : GND, 3V3, SDA, SCL

Make sure that the device is available at `0x68` or `0x69` i2c address by running this command:

`i2cdetect -y 1`

## Tested with:
- Ubuntu 22.04.2 LTS
- Raspbian Buster (32 Bit)

## Dependencies

### H5py (for data plotting)
Ubuntu:
`pip3 install h5py`

Raspbian:
`sudo apt-get install python3-h5py`