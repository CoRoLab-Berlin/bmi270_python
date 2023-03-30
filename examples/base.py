import h5py
import threading
import time

from bmi270.BMI270 import *
from bmi270.UDP import *

# from src.bmi270.BMI270 import *
# from src.bmi270.UDP import *

# -------------------------------------------------
# INITIALIZATION
# -------------------------------------------------
BMI270_1 = BMI270(I2C_PRIM_ADDR)
BMI270_1.load_config_file()

BMI270_2 = BMI270(I2C_SEC_ADDR)
BMI270_2.load_config_file()

# -------------------------------------------------
# HARDWARE CONFIGURATION
# -------------------------------------------------

BMI270_1.set_mode(PERFORMANCE_MODE)
BMI270_1.set_acc_range(ACC_RANGE_2G)
BMI270_1.set_gyr_range(GYR_RANGE_2000)
BMI270_1.set_acc_odr(ACC_ODR_200)
BMI270_1.set_gyr_odr(GYR_ODR_200)
BMI270_1.set_acc_bwp(ACC_BWP_OSR4)
BMI270_1.set_gyr_bwp(GYR_BWP_OSR4)
BMI270_1.enable_fifo_streaming()
BMI270_1.disable_fifo_header()

BMI270_2.set_mode(PERFORMANCE_MODE)
BMI270_2.set_acc_range(ACC_RANGE_2G)
BMI270_2.set_gyr_range(GYR_RANGE_2000)
BMI270_2.set_acc_odr(ACC_ODR_200)
BMI270_2.set_gyr_odr(GYR_ODR_200)
BMI270_2.set_acc_bwp(ACC_BWP_OSR4)
BMI270_2.set_gyr_bwp(GYR_BWP_OSR4)
BMI270_2.enable_fifo_streaming()
BMI270_2.disable_fifo_header()

# -------------------------------------------------
# NETWORK CONFIGURATION
# -------------------------------------------------

RECEIVER_IP = ''
RECEIVER_PORT = 12345
SENDER_IP = ''
SENDER_PORT = 12345

Network = UDP(RECEIVER_IP, RECEIVER_PORT, SENDER_IP, SENDER_PORT)

# -------------------------------------------------
# FUNCTIONS
# -------------------------------------------------


def print_seconds():
    threading.Timer(1.0, print_seconds).start()
    print("-" * 80, " ", int(time.time() - start_time), "s")


def UDP_send_data():
    # threading.Timer(HERTZ_100, UDP_send_data).start()
    data_bytes = Network.pack_data(BMI270_1.get_sensor_time(), BMI270_1.get_raw_acc_data(), BMI270_1.get_raw_gyr_data(),
                                   BMI270_2.get_sensor_time(), BMI270_2.get_raw_acc_data(), BMI270_2.get_raw_gyr_data())
    Network.send_data(data_bytes)
    sleep(HERTZ_200)


# -------------------------------------------------
# MAIN
# -------------------------------------------------

print("\nStarting in 3 seconds...")
time.sleep(3)
start_time = time.time()

def main():
    print_seconds()

    while True:
        UDP_send_data()

if __name__ == "__main__":
    main()




# -------------------------------------------------
# H5 FILE
# -------------------------------------------------

# # Save as H5 file
# if False:
#     print("---- SAVING FILE ----")
#     with h5py.File("sensor_data.h5", "w") as h5_file:
#         h5_file.create_dataset("accel_data_1", data=accel_array_1)
#         h5_file.create_dataset("gyro_data_1", data=gyro_array_1)
#         h5_file.create_dataset("accel_data_2", data=accel_array_2)
#         h5_file.create_dataset("gyro_data_2", data=gyro_array_2)
#     print("---- FILE SAVED ----")
