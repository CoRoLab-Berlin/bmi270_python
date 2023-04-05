#!/usr/bin/env python3

import numpy as np
import socket
import struct
import time
import quaternion
from vqf import VQF

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion


# from bmi270.definitions import *

from src.bmi270.definitions import *


# -------------------------------------------------
# CONSTANTS
# -------------------------------------------------

# Adjust these constants to your sensor setup
IMU_ACC_RANGE = 2 * GRAVITY     # equivalent to 2g
IMU_GYR_RANGE = 125


# -------------------------------------------------
# NETWORK CONFIGURATION
# -------------------------------------------------

RECEIVER_ADDRESS = ('0.0.0.0', 8000)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(RECEIVER_ADDRESS)
sock.settimeout(5)


# -------------------------------------------------
# PUBLISHER
# -------------------------------------------------

class PneumaticArmIMUPublisher(Node):
    def __init__(self):
        super().__init__('QuaternionPublisher')
        self.publisher_upper = self.create_publisher(Quaternion, 'pneumatic_arm/IMU_upper', 10)
        self.publisher_lower = self.create_publisher(Quaternion, 'pneumatic_arm/IMU_lower', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.data_stream = False
        self.old_time_1 = 0
        self.old_time_2 = 0
        self.q1 = quaternion.one
        self.q2 = quaternion.one
        print("Publisher started...")

    def receive_data(self):
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            self.data_stream = False
            print("ERROR: No data received! Trying again in 5 seconds...")
            return

        self.data_stream = True
        data = struct.unpack('14i', data)
    
        # Data processing:
        data = self.quaternion_orientation(data)
        # data = self.vqf_orientation(data)
        
        return data

    def publish_imu_data(self):
        quaternion_data = self.receive_data()

        if self.data_stream:

            # IMU_1 (upper) - Quaternions
            msg_quat_1 = Quaternion()
            msg_quat_1.w = quaternion_data[0]
            msg_quat_1.x = quaternion_data[1]
            msg_quat_1.y = quaternion_data[2]
            msg_quat_1.z = quaternion_data[3]

            # IMU_2 (lower) - Quaternions
            msg_quat_2 = Quaternion()
            msg_quat_2.w = quaternion_data[4]
            msg_quat_2.x = quaternion_data[5]
            msg_quat_2.y = quaternion_data[6]
            msg_quat_2.z = quaternion_data[7]

            # Publish to topics
            self.publisher_upper.publish(msg_quat_1)
            self.publisher_lower.publish(msg_quat_2)

            # IMU_1 (upper) - Transform
            t1 = TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg()
            t1.header.frame_id = "map"
            t1.child_frame_id = "pneumatic_arm_IMU_upper_link"
            t1.transform.rotation = msg_quat_1
            t1.transform.translation.z = 1.0

            # IMU_2 (lower) - Transform
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg()
            t2.header.frame_id = "pneumatic_arm_IMU_upper_link"
            t2.child_frame_id = "pneumatic_arm_IMU_lower_link"
            t2.transform.rotation = msg_quat_2
            t2.transform.translation.x += 1.0

            # Joint - Transform
            joint = TransformStamped()
            joint.header.stamp = self.get_clock().now().to_msg()
            joint.header.frame_id = "pneumatic_arm_IMU_lower_link"
            joint.child_frame_id = "end_link"
            joint.transform.translation.x = 1.0

            # Publish to TF
            self.tf_broadcaster.sendTransform(t1)
            self.tf_broadcaster.sendTransform(t2)
            self.tf_broadcaster.sendTransform(joint)

    def quaternion_orientation(self, data):
        # IMU_1
        new_time_1 = data[0]
        dt_1 = (new_time_1 - self.old_time_1) / 1000
        self.old_time_1 = new_time_1

        acc_1 = np.array([data[1], data[2], data[3]]) / 32767 * IMU_ACC_RANGE                           # in m/s²
        gyr_1 = np.deg2rad(1) * np.array([data[4], data[5], data[6]]) / 32767 * IMU_GYR_RANGE / 2       # in rad/s

        q_dot_1 = quaternion.quaternion(0, gyr_1[0], gyr_1[1], gyr_1[2])
        self.q1 += q_dot_1 * self.q1 * dt_1
        self.q1 = self.q1.normalized()

        # IMU_2
        new_time_2 = data[7]
        dt_2 = (new_time_2 - self.old_time_2) / 1000
        self.old_time_2 = new_time_2

        acc2 = np.array([data[8], data[9], data[10]]) / 32767 * IMU_ACC_RANGE                          # in m/s²
        gyr2 = np.deg2rad(1) * np.array([data[11], data[12], data[13]]) / 32767 * IMU_GYR_RANGE / 2    # in rad/s

        q_dot_2 = quaternion.quaternion(0, gyr2[0], gyr2[1], gyr2[2])
        self.q2 += q_dot_2 * self.q2 * dt_2
        self.q2 = self.q2.normalized()

        return np.concatenate((self.q1.components, self.q2.components))

    def vqf_orientation(self, data):
        # BMI270_2
        new_time = data[7]
        Ts = (new_time - self.old_time) / 1000
        self.old_time = new_time

        acc = np.array([data[8], data[9], data[10]]) / 32767 * IMU_ACC_RANGE                    # in m/s²
        gyr = np.deg2rad(1) * np.array([data[11], data[12], data[13]]) / 32767 * IMU_GYR_RANGE  # in rad/s

        vqf = VQF(Ts, magDistRejectionEnabled=False)

        vqf.update(acc, gyr)

        return vqf.getQuat6D()


# -------------------------------------------------
# HELPER FUNCTIONS
# -------------------------------------------------


# -------------------------------------------------
# MAIN
# -------------------------------------------------
start_time = time.time()  # in seconds


def main():

    rclpy.init()
    my_publisher = PneumaticArmIMUPublisher()

    while True:
        my_publisher.publish_imu_data()

    # rclpy.spin(my_publisher)()
    my_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
