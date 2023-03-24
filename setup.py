import setuptools

with open('README.md', 'r') as fh:
    long_description = fh.read()

setuptools.setup(
  name='bmi270_python',
  version='1.0',
  author='Kevin Sommler - ksomml',
  author_email='sommler@live.de',
  description="BMI270 I2C Python library",
  long_description=long_description,
  long_description_content_type='text/markdown',
  url='https://github.com/CoRoLab-Berlin/bmi270_python',
  packages=setuptools.find_packages(),
  install_requires=['smbus2'],
  keywords="BMI270 I2C driver IMU sensor acclerometer gyroscope gyro bosch sensortech smbus2 library",
  classifiers=[
    'Programming Language :: Python :: 3',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Libraries',
    'Topic :: System :: Hardware',
  ],
  python_requires='>=3.6',
)
