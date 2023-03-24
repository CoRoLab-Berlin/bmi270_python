import h5py

import BMI270_I2C

# -------------------------------------------------
# INITIALIZATION
# -------------------------------------------------
BMI270_1 = BMI270(I2C_PRIM_ADDR)
BMI270_1.load_config_file()

BMI270_2 = BMI270(I2C_SEC_ADDR)
BMI270_2.load_config_file()

# -------------------------------------------------
# CONFIGURATION
# -------------------------------------------------

BMI270_1.set_mode(PERFORMANCE_MODE)
BMI270_1.set_acc_range(ACC_RANGE_2G)
BMI270_1.set_gyr_range(GYR_RANGE_2000)
BMI270_1.set_acc_odr(ACC_ODR_200)
BMI270_1.set_gyr_odr(GYR_ODR_200)
BMI270_1.set_acc_bwp(ACC_BWP_NORMAL)
BMI270_1.set_gyr_bwp(GYR_BWP_NORMAL)

BMI270_2.set_mode(PERFORMANCE_MODE)
BMI270_2.set_acc_range(ACC_RANGE_2G)
BMI270_2.set_gyr_range(GYR_RANGE_2000)
BMI270_2.set_acc_odr(ACC_ODR_200)
BMI270_2.set_gyr_odr(GYR_ODR_200)
BMI270_2.set_acc_bwp(ACC_BWP_OSR4)
BMI270_2.set_gyr_bwp(GYR_BWP_OSR4)


# -------------------------------------------------
# DATA ACQUISITION
# -------------------------------------------------

timer = 0
accel_array_1 = np.zeros((1, 3))
gyro_array_1 = np.zeros((1, 3))
accel_array_2 = np.zeros((1, 3))
gyro_array_2 = np.zeros((1, 3))

try:
    while True:
            accel_data_1 = BMI270_1.get_acc_data()
            gyro_data_1 = BMI270_1.get_gyr_data()
            accel_data_2 = BMI270_2.get_acc_data()
            gyro_data_2 = BMI270_2.get_gyr_data()

            accel_array_1 = np.append(accel_array_1, accel_data_1, axis=0)
            gyro_array_1 = np.append(gyro_array_1, gyro_data_1, axis=0)
            accel_array_2 = np.append(accel_array_2, accel_data_2, axis=0)
            gyro_array_2 = np.append(gyro_array_2, gyro_data_2, axis=0)

            # print(f'1-ACCEL: {accel_data_1}'.ljust(33), f'1-GYRO: {gyro_data_1}'.ljust(33))
            print(f'1-ACCEL: {accel_data_1}'.ljust(33), f'1-GYRO: {gyro_data_1}'.ljust(33), f'2-ACCEL: {accel_data_2}'.ljust(33), f'GYRO: {gyro_data_2}'.ljust(33))

            sleep(0.02)
            timer += 1
            if (timer % 10 == 0):
                print("-" * 130, int(timer/10), "s")
except KeyboardInterrupt:
    print("\n---- DATA STOPPED - WRITING FILE... ----")
    pass

# Save as H5 file
if False:
    with h5py.File("sensor_data.h5", "w") as h5_file:
        h5_file.create_dataset("accel_data_1", data=accel_array_1)
        h5_file.create_dataset("gyro_data_1", data=gyro_array_1)
        h5_file.create_dataset("accel_data_2", data=accel_array_2)
        h5_file.create_dataset("gyro_data_2", data=gyro_array_2)

print("---- FILE SAVED ----")

