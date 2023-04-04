#!/usr/bin/env python3

import numpy as np
# import qmt
import socket
import struct
import threading
import time
import quaternion

import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion


from bmi270.definitions import *

# from src.bmi270.definitions import *

# -------------------------------------------------
# CONSTANTS
# -------------------------------------------------

CURRENT_ACC_RANGE = 2
CURRENT_GYR_RANGE = 125


# -------------------------------------------------
# PUBLISHER
# -------------------------------------------------


class MyPublisher(Node):
    def __init__(self):
        super().__init__('QuaternionPublisher')
        self.publisher_ = self.create_publisher(Quaternion, 'quaternions', 10)
        self.tf_broadcaster_ = tf2_ros.StaticTransformBroadcaster(self)
        self.old_time = 0

    def timer_callback(self):
        quaternion_data = self.receive_data()
        
        msg = Quaternion()
        msg.x = quaternion_data[1]
        msg.y = quaternion_data[2]
        msg.z = quaternion_data[3]
        msg.w = quaternion_data[0]

        self.publisher_.publish(msg)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "map"
        transform.transform.rotation = msg

        self.tf_broadcaster_.sendTransform(transform)

    def receive_data(self):
        data, addr = sock.recvfrom(1024)
        data = struct.unpack('14i', data)
        data = self.quaternion_orientation(data)
        return data

    def quaternion_orientation(self, data):
        q = quaternion.one

        new_time = data[7]
        dt = (new_time - self.old_time) / 1000
        self.old_time = new_time

        # Raw sensor data
        accelerometer_data = np.array([data[8], data[9], data[10]])
        gyroscope_data = np.array([data[11], data[12], data[13]])

        # Convert accelerometer data to m/s^2
        acc_x = (accelerometer_data[0] / 32767) * CURRENT_ACC_RANGE * GRAVITY
        acc_y = (accelerometer_data[1] / 32767) * CURRENT_ACC_RANGE * GRAVITY
        acc_z = (accelerometer_data[2] / 32767) * CURRENT_ACC_RANGE * GRAVITY

        # Convert gyroscope data to radians/s
        gyr_x = (gyroscope_data[0] / 32767) * CURRENT_GYR_RANGE * DEG2RAD / 2
        gyr_y = (gyroscope_data[1] / 32767) * CURRENT_GYR_RANGE * DEG2RAD / 2
        gyr_z = (gyroscope_data[2] / 32767) * CURRENT_GYR_RANGE * DEG2RAD / 2

        # Calculate quaternion orientation
        q_dot = quaternion.quaternion(0, gyr_x, gyr_y, gyr_z)
        q += q_dot * q * dt
        q = q.normalized()

        return np.array(q.components)


# -------------------------------------------------
# NETWORK CONFIGURATION
# -------------------------------------------------

RECEIVER_ADDRESS = ('0.0.0.0', 8000)
SENDER_PORT = 8000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(RECEIVER_ADDRESS)


# -------------------------------------------------
# MAIN
# -------------------------------------------------


def print_seconds():
    threading.Timer(1.0, print_seconds).start()
    print("-" * 80, " ", int(time.time() - start_time), "s")


start_time = time.time()


def main():
    print_seconds()

    rclpy.init()
    my_publisher = MyPublisher()
    
    while True:
        MyPublisher.timer_callback(my_publisher)

    # rclpy.spin(my_publisher)()
    my_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
