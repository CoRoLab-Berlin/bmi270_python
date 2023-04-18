# BMI270 I2C Python Implementation - Version: 0.4.0

Bare bones BMI270 I2C Python implementation. This was a project for my practical phase at my University.

## Installation
The package is [available on pypi.org](https://pypi.org/project/bmi270).

You can install this package using this command:

`pip3 install bmi270`

For a Raspberry Pi Setup add/change this line in /boot/config.txt to your desired baudrate:

`dtparam=i2c_baudrate=400000`

Reboot your Raspberry Pi after applying the change.

## Usage
Make sure these lines are connected: GND, 3V3, SDA, SCL

If you are using the SparkFun SPX-17353 BMI270 Breakout Board, you can easily connect them using a 1mm 4-pin JST connector cable. (example: Qwiic cables)

Make sure that the device is available at `0x68` or `0x69` i2c address by running this command:

`i2cdetect -y 1`

The BMI270 requires a config load. This initialization step is necessary to be able to use all its functions.

`load_config()`

A full power cycle is necessary if you want to load the config again.

Check out [examples](https://github.com/CoRoLab-Berlin/bmi270_python/tree/main/examples) for more information.

## Tested with:
- Ubuntu 22.04.2 LTS
- Raspbian 10 - Buster (32 Bit)

## Dependencies

**requires Python >= 3.7**

**[smbus2](https://github.com/kplindegaard/smbus2)** *(auto installed)*

Manual installation:
`pip3 install smbus2`

## Functionality

- BMI270 class integration
- load config file into BMI270
- write/read registers
- a few other functions (see [BMI270.py](https://github.com/CoRoLab-Berlin/bmi270_python/blob/main/src/bmi270/BMI270.py))

## Credits & Related links

- [serioeseGmbH/BMI160](https://github.com/serioeseGmbH/BMI160)
- [lefuturiste](https://github.com/lefuturiste/BMI160-i2c)
- [BMI270 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
- [smbus2 docs](https://smbus2.readthedocs.io/en/latest/)

## Troubleshooting

-bash: pip3: command not found

`sudo apt install python3-pip`