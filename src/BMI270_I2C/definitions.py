# -------------------------------------------------
# DEFINITIONS
# -------------------------------------------------

# General
GRAVITY         = 9.80725
BIT_8           = 8
BIT_16          = 16
BIT_32          = 32
BINARY          = "bin"
HEXADECIMAL     = "hex"
LSB_MASK_8BIT   = 0x0F      # 00001111
MSB_MASK_8BIT   = 0xF0      # 11110000
LSB_8_MASK_8BIT = 0x8F      # 10001111

# Device Modes
LOW_POWER_MODE = "low_power"
NORMAL_MODE = "normal"
PERFORMANCE_MODE = "performance"

# Accelerometer
ACC_RANGE_2G    = 0x00      # +/- 2g
ACC_RANGE_4G    = 0x01      # +/- 4g
ACC_RANGE_8G    = 0x02      # +/- 8g
ACC_RANGE_16G   = 0x03      # +/- 16g
ACC_ODR_1600    = 0x0C      # 1600Hz
ACC_ODR_800     = 0x0B      # 800Hz
ACC_ODR_400     = 0x0A      # 400Hz
ACC_ODR_200     = 0x09      # 200Hz
ACC_ODR_100     = 0x08      # 100Hz
ACC_ODR_50      = 0x07      # 50Hz
ACC_ODR_25      = 0x06      # 25Hz
ACC_BWP_OSR4    = 0x00      # OSR4
ACC_BWP_OSR2    = 0x01      # OSR2
ACC_BWP_NORMAL  = 0x02      # Normal
ACC_BWP_CIC     = 0x03      # CIC
ACC_BWP_RES16   = 0x04      # Reserved
ACC_BWP_RES32   = 0x05      # Reserved
ACC_BWP_RES64   = 0x06      # Reserved
ACC_BWP_RES128  = 0x07      # Reserved

# Gyroscope
GYR_RANGE_2000  = 0x00      # +/- 2000dps,  16.4 LSB/dps
GYR_RANGE_1000  = 0x01      # +/- 1000dps,  32.8 LSB/dps
GYR_RANGE_500   = 0x02      # +/- 500dps,   65.6 LSB/dps
GYR_RANGE_250   = 0x03      # +/- 250dps,  131.2 LSB/dps
GYR_RANGE_125   = 0x04      # +/- 125dps,  262.4 LSB/dps
GYR_ODR_3200    = 0x0D      # 3200Hz
GYR_ODR_1600    = 0x0C      # 1600Hz
GYR_ODR_800     = 0x0B      # 800Hz
GYR_ODR_400     = 0x0A      # 400Hz
GYR_ODR_200     = 0x09      # 200Hz
GYR_ODR_100     = 0x08      # 100Hz
GYR_ODR_50      = 0x07      # 50Hz
GYR_ODR_25      = 0x06      # 25Hz
GYR_BWP_OSR4    = 0x00      # OSR4
GYR_BWP_OSR2    = 0x01      # OSR2
GYR_BWP_NORMAL  = 0x02      # Normal