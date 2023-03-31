import numpy as np
from smbus2 import SMBus
from time import sleep

# from bmi270.config_file import *
# from bmi270.registers import *
# from bmi270.definitions import *

from src.bmi270.config_file import *
from src.bmi270.registers import *
from src.bmi270.definitions import *

class BMI270:
    def __init__(self, i2c_addr) -> None:
        self.bus = SMBus(I2C_BUS)
        if (self.bus == -1):
            print("---- ERROR: I2C BUS NOT FOUND ----")
            exit(1)
        else:
            print("---- I2C BUS FOUND ----")
        self.address        = i2c_addr
        print(hex(self.address), " --> Chip ID: " + hex(self.bus.read_byte_data(i2c_addr, CHIP_ID_ADDRESS)))
        self.acc_range        = 2 * GRAVITY
        self.acc_odr          = 100
        self.gyr_range        = 2000
        self.gyr_odr          = 200

    def read_register(self, register_address) -> int:
            return self.bus.read_byte_data(self.address, register_address)

    def write_register(self, register_address, byte_data) -> None:
            self.bus.write_byte_data(self.address, register_address, byte_data)

    def load_config_file(self) -> None:
        if (self.read_register(INTERNAL_STATUS) == 0x01):
            print(hex(self.address), " --> Initialization already done")
        else:
            print(hex(self.address), " --> Initializing...")
            self.write_register(PWR_CONF, 0x00)
            sleep(0.00045)
            self.write_register(INIT_CTRL, 0x00)
            for i in range(256):
                self.write_register(INIT_ADDR_0, 0x00)
                self.write_register(INIT_ADDR_1, i)
                self.bus.write_i2c_block_data(self.address, INIT_DATA, bmi270_config_file[i*32:(i+1)*32])
                sleep(0.000020)
            self.write_register(INIT_CTRL, 0x01)
            sleep(0.02)
        print(hex(self.address), " --> Initialization status: " + '{:08b}'.format(self.read_register(INTERNAL_STATUS)) + "\t(00000001 --> OK)")

    def set_mode(self, mode="performance") -> None:
        if (mode == "low_power"):
            self.write_register(PWR_CTRL, 0x04)
            self.write_register(ACC_CONF, 0x17)
            self.write_register(GYR_CONF, 0x28)
            self.write_register(PWR_CONF, 0x03)
            self.acc_odr = 50
            self.gyr_odr = 100
            print(hex(self.address), " --> Mode set to: LOW_POWER_MODE")
        elif (mode == "normal"):
            self.write_register(PWR_CTRL, 0x0E)
            self.write_register(ACC_CONF, 0xA8)
            self.write_register(GYR_CONF, 0xA9)
            self.write_register(PWR_CONF, 0x02)
            self.acc_odr = 100
            self.gyr_odr = 200
            print(hex(self.address), " --> Mode set to: NORMAL_MODE")
        elif (mode == "performance"):
            self.write_register(PWR_CTRL, 0x0E)
            self.write_register(ACC_CONF, 0xA8)
            self.write_register(GYR_CONF, 0xE9)
            self.write_register(PWR_CONF, 0x02)
            self.acc_odr = 100
            self.gyr_odr = 200
            print(hex(self.address), " --> Mode set to: PERFORMANCE_MODE")
        else:
            print("Wrong mode. Use 'low_power', 'normal' or 'performance'")

    def print_read_register(self, register_address, format="bin") -> None:
        if (format == "bin"):
            data = self.read_register(register_address)
            print("Register " + hex(register_address) + ": " + '{:08b}'.format(data))
        elif (format == "hex"):
            data = self.read_register(register_address)
            print("Register " + hex(register_address) + ": " + hex(data))
        else:
            print("Wrong format. Use 'hex' or 'bin'")

    def print_write_register(self, register_address, byte_data) -> None:
        print(hex(register_address) + " before: \t" + '{:08b}'.format(self.read_register(register_address)))
        self.bus.write_byte_data(self.address, register_address, byte_data)
        print(hex(register_address) + " after: \t" + '{:08b}'.format(self.read_register(register_address)))

    def enable_fifo_streaming(self) -> None:
        self.write_register(FIFO_CONFIG_1, ((self.read_register(FIFO_CONFIG_1) & FIRST_5_BITS) | LAST_3_BITS))
        print(hex(self.address), " --> FIFO Streaming Mode enabled")

    def disable_fifo_streaming(self) -> None:
        self.write_register(FIFO_CONFIG_1, (self.read_register(FIFO_CONFIG_1) & FIRST_5_BITS))
        print(hex(self.address), " --> FIFO Streaming Mode disabled")

    def enable_fifo_header(self) -> None:
        self.write_register(FIFO_CONFIG_1, ((self.read_register(FIFO_CONFIG_1) & (LSB_MASK_8BIT | LAST_3_BITS)) | BIT_4))
        print(hex(self.address), " --> FIFO Header enabled")

    def disable_fifo_header(self) -> None:
        self.write_register(FIFO_CONFIG_1, ((self.read_register(FIFO_CONFIG_1) & (LSB_MASK_8BIT | LAST_3_BITS))))
        print(hex(self.address), " --> FIFO Header disabled")

    def enable_aux(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) | BIT_0)

    def disable_aux(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) & ~BIT_0)

    def enable_gyr(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) | BIT_1)

    def disable_gyr(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) & ~BIT_1)

    def enable_acc(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) | BIT_2)

    def disable_acc(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) & ~BIT_2)

    def enable_temp(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) | BIT_3)

    def disable_temp(self) -> None:
        self.write_register(PWR_CTRL, (self.read_register(PWR_CTRL) & LSB_MASK_8BIT) & ~BIT_3)

    def set_acc_range(self, range=ACC_RANGE_2G) -> None:
        if (range == ACC_RANGE_2G):
            self.write_register(ACC_RANGE, ACC_RANGE_2G)
            self.acc_range = 2 * GRAVITY
            print(hex(self.address), " --> ACC range set to: 2G")
        elif (range == ACC_RANGE_4G):
            self.write_register(ACC_RANGE, ACC_RANGE_4G)
            self.acc_range = 4 * GRAVITY
            print(hex(self.address), " --> ACC range set to: 4G")
        elif (range == ACC_RANGE_8G):
            self.write_register(ACC_RANGE, ACC_RANGE_8G)
            self.acc_range = 8 * GRAVITY
            print(hex(self.address), " --> ACC range set to: 8G")
        elif (range == ACC_RANGE_16G):
            self.write_register(ACC_RANGE, ACC_RANGE_16G)
            self.acc_range = 16 * GRAVITY
            print(hex(self.address), " --> ACC range set to: 16G")
        else:
            print("Wrong range. Use 'ACC_RANGE_2G', 'ACC_RANGE_4G', 'ACC_RANGE_8G' or 'ACC_RANGE_16G'")

    def set_gyr_range(self, range=GYR_RANGE_2000) -> None:
        if (range == GYR_RANGE_2000):
            self.write_register(GYR_RANGE, GYR_RANGE_2000)
            self.gyr_range = 2000
            print(hex(self.address), " --> GYR range set to: 2000")
        elif (range == GYR_RANGE_1000):
            self.write_register(GYR_RANGE, GYR_RANGE_1000)
            self.gyr_range = 1000
            print(hex(self.address), " --> GYR range set to: 1000")
        elif (range == GYR_RANGE_500):
            self.write_register(GYR_RANGE, GYR_RANGE_500)
            self.gyr_range = 500
            print(hex(self.address), " --> GYR range set to: 500")
        elif (range == GYR_RANGE_250):
            self.write_register(GYR_RANGE, GYR_RANGE_250)
            self.gyr_range = 250
            print(hex(self.address), " --> GYR range set to: 250")
        elif (range == GYR_RANGE_125):
            self.write_register(GYR_RANGE, GYR_RANGE_125)
            self.gyr_range = 125
            print(hex(self.address), " --> GYR range set to: 125")
        else:
            print("Wrong range. Use 'GYR_RANGE_2000', 'GYR_RANGE_1000', 'GYR_RANGE_500', 'GYR_RANGE_250' or 'GYR_RANGE_125'")

    def set_acc_odr(self, odr=ACC_ODR_200) -> None:
        if (odr == ACC_ODR_1600):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_1600))
            self.acc_odr = 1600
            print(hex(self.address), " --> ACC ODR set to: 1600")
        elif (odr == ACC_ODR_800):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_800))
            self.acc_odr = 800
            print(hex(self.address), " --> ACC ODR set to: 800")
        elif (odr == ACC_ODR_400):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_400))
            self.acc_odr = 400
            print(hex(self.address), " --> ACC ODR set to: 400")
        elif (odr == ACC_ODR_200):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_200))
            self.acc_odr = 200
            print(hex(self.address), " --> ACC ODR set to: 200")
        elif (odr == ACC_ODR_100):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_100))
            self.acc_odr = 100
            print(hex(self.address), " --> ACC ODR set to: 100")
        elif (odr == ACC_ODR_50):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_50))
            self.acc_odr = 50
            print(hex(self.address), " --> ACC ODR set to: 50")
        elif (odr == ACC_ODR_25):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & MSB_MASK_8BIT) | ACC_ODR_25))
            self.acc_odr = 25
            print(hex(self.address), " --> ACC ODR set to: 25")

    def set_gyr_odr(self, odr=GYR_ODR_200) -> None:
        if (odr == GYR_ODR_3200):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_3200))
            self.gyr_odr = 3200
            print(hex(self.address), " --> GYR ODR set to: 3200")
        elif (odr == GYR_ODR_1600):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_1600))
            self.gyr_odr = 1600
            print(hex(self.address), " --> GYR ODR set to: 1600")
        elif (odr == GYR_ODR_800):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_800))
            self.gyr_odr = 800
            print(hex(self.address), " --> GYR ODR set to: 800")
        elif (odr == GYR_ODR_400):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_400))
            self.gyr_odr = 400
            print(hex(self.address), " --> GYR ODR set to: 400")
        elif (odr == GYR_ODR_200):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_200))
            self.gyr_odr = 200
            print(hex(self.address), " --> GYR ODR set to: 200")
        elif (odr == GYR_ODR_100):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_100))
            self.gyr_odr = 100
            print(hex(self.address), " --> GYR ODR set to: 100")
        elif (odr == GYR_ODR_50):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_50))
            self.gyr_odr = 50
            print(hex(self.address), " --> GYR ODR set to: 50")
        elif (odr == GYR_ODR_25):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & MSB_MASK_8BIT) | GYR_ODR_25))
            self.gyr_odr = 25
            print(hex(self.address), " --> GYR ODR set to: 25")

    def set_acc_bwp(self, bwp=ACC_BWP_NORMAL) -> None:
        if (bwp == ACC_BWP_OSR4):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_OSR4 << 4)))
            print(hex(self.address), " --> ACC BWP set to: OSR4")
        elif (bwp == ACC_BWP_OSR2):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_OSR2 << 4)))
            print(hex(self.address), " --> ACC BWP set to: OSR2")
        elif (bwp == ACC_BWP_NORMAL):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_NORMAL << 4)))
            print(hex(self.address), " --> ACC BWP set to: NORMAL")
        elif (bwp == ACC_BWP_CIC):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_CIC << 4)))
            print(hex(self.address), " --> ACC BWP set to: CIC")
        elif (bwp == ACC_BWP_RES16):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES16 << 4)))
            print(hex(self.address), " --> ACC BWP set to: RES16")
        elif (bwp == ACC_BWP_RES32):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES32 << 4)))
            print(hex(self.address), " --> ACC BWP set to: RES32")
        elif (bwp == ACC_BWP_RES64):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES64 << 4)))
            print(hex(self.address), " --> ACC BWP set to: RES64")
        elif (bwp == ACC_BWP_RES128):
            self.write_register(ACC_CONF, ((self.read_register(ACC_CONF) & LSB_MASK_8BIT_8) | (ACC_BWP_RES128 << 4)))
            print(hex(self.address), " --> ACC BWP set to: RES128")

    def set_gyr_bwp(self, bwp=GYR_BWP_NORMAL) -> None:
        if (bwp == GYR_BWP_OSR4):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_OSR4 << 4)))
            print(hex(self.address), " --> GYR BWP set to: OSR4")
        elif (bwp == GYR_BWP_OSR2):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_OSR2 << 4)))
            print(hex(self.address), " --> GYR BWP set to: OSR2")
        elif (bwp == GYR_BWP_NORMAL):
            self.write_register(GYR_CONF, ((self.read_register(GYR_CONF) & LSB_MASK_8BIT_8) | (GYR_BWP_NORMAL << 4)))
            print(hex(self.address), " --> GYR BWP set to: NORMAL")

    def get_sensor_time(self) -> int:
        sensortime_0 = self.read_register(SENSORTIME_0)
        sensortime_1 = self.read_register(SENSORTIME_1)
        sensortime_2 = self.read_register(SENSORTIME_2)

        return (sensortime_2 << 16) | (sensortime_1 << 8) | sensortime_0

    def get_raw_acc_data(self) -> np.ndarray:
        acc_value_x_lsb = self.read_register(ACC_X_7_0)
        acc_value_x_msb = self.read_register(ACC_X_15_8)
        acc_value_x = (acc_value_x_msb << 8) | acc_value_x_lsb

        acc_value_y_lsb = self.read_register(ACC_Y_7_0)
        acc_value_y_msb = self.read_register(ACC_Y_15_8)
        acc_value_y = (acc_value_y_msb << 8) | acc_value_y_lsb

        acc_value_z_lsb = self.read_register(ACC_Z_7_0)
        acc_value_z_msb = self.read_register(ACC_Z_15_8)
        acc_value_z = (acc_value_z_msb << 8) | acc_value_z_lsb

        return np.array([acc_value_x, acc_value_y, acc_value_z]).astype(np.int16)

    def get_raw_gyr_data(self) -> np.ndarray:
        gyr_value_x_lsb = self.read_register(GYR_X_7_0)
        gyr_value_x_msb = self.read_register(GYR_X_15_8)
        gyr_value_x = (gyr_value_x_msb << 8) | gyr_value_x_lsb  # - GYR_CAS.factor_zx * (gyr_value_z_msb << 8 | gyr_value_z_lsb) / 2**9

        gyr_value_y_lsb = self.read_register(GYR_Y_7_0)
        gyr_value_y_msb = self.read_register(GYR_Y_15_8)
        gyr_value_y = (gyr_value_y_msb << 8) | gyr_value_y_lsb

        gyr_value_z_lsb = self.read_register(GYR_Z_7_0)
        gyr_value_z_msb = self.read_register(GYR_Z_15_8)
        gyr_value_z = (gyr_value_z_msb << 8) | gyr_value_z_lsb

        return np.array([gyr_value_x, gyr_value_y, gyr_value_z]).astype(np.int16)
    
    def get_raw_temp_data(self) -> np.ndarray:
        temp_value_lsb = self.read_register(TEMPERATURE_0)
        temp_value_msb = self.read_register(TEMPERATURE_1)
        temp_value = (temp_value_msb << 8) | temp_value_lsb

        return np.array([temp_value]).astype(np.int16)