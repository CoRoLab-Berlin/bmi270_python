import numpy as np
import socket
import struct
import threading
import time
# import qmt
import quaternion

import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion


# from bmi270.definitions import *

from src.bmi270.definitions import *

# -------------------------------------------------
# PUBLISHER
# -------------------------------------------------


class MyPublisher(Node):
    def __init__(self):
        super().__init__('QuaternionPublisher')
        self.publisher_ = self.create_publisher(Quaternion, 'quaternions', 10)
        self.tf_broadcaster_ = tf2_ros.StaticTransformBroadcaster(self)
        self.i = 0

    def timer_callback(self):
        quaternion_data = self.receive_data()
        msg = Quaternion()
        # msg.x = quaternion_data[1]
        # msg.y = quaternion_data[2]
        # msg.z = quaternion_data[3]
        # msg.w = quaternion_data[0]

        self.i += 1

        # self._logger.info(f"Publishing {self.i}")
        self.publisher_.publish(msg)

        # create a transform based on the quaternion
        # transform = TransformStamped()
        # transform.header.stamp = self.get_clock().now().to_msg()
        # transform.header.frame_id = "world"
        # transform.child_frame_id = "map"
        # transform.transform.rotation = msg

        # broadcast the transform
        # self.tf_broadcaster_.sendTransform(transform)

    def receive_data(self):
        data, addr = sock.recvfrom(1024)
        # data = struct.unpack('14i', data)
        # data = self.quaternion_orientation(data)
        return data

    def quaternion_orientation(self, data):
        # Define initial variables
        q = quaternion.one
        dt = 0.005  # time step in seconds

        # Raw sensor data
        accelerometer_data = np.array([data[8], data[9], data[10]])
        gyroscope_data = np.array([data[11], data[12], data[13]])

        # Convert accelerometer data to m/s^2
        acceleration = np.array([0, 0, 0]).astype(np.float64)
        acceleration[0] = (accelerometer_data[0] / 32767) * 2 * GRAVITY
        acceleration[1] = (accelerometer_data[1] / 32767) * 2 * GRAVITY
        acceleration[2] = (accelerometer_data[2] / 32767) * 2 * GRAVITY

        # Convert gyroscope data to radians/s
        angular_velocity = np.array([0, 0, 0]).astype(np.float64)
        angular_velocity[0] = (gyroscope_data[0] / 32767) * 2000 * DEG2RAD
        angular_velocity[1] = (gyroscope_data[1] / 32767) * 2000 * DEG2RAD
        angular_velocity[2] = (gyroscope_data[2] / 32767) * 2000 * DEG2RAD

        # Calculate quaternion orientation
        q_dot = quaternion.quaternion(0, angular_velocity[0], angular_velocity[1], angular_velocity[2])
        q += q_dot * q * dt
        q = q.normalized()

        return np.array(q.components, dtype=np.float64)

# -------------------------------------------------
# DEFINES
# -------------------------------------------------


UDP_IP = '0.0.0.0'
UDP_RECEIVER_PORTS = 33771
UDP_SENDER_PORT = 33772


# -------------------------------------------------
# NETWORK CONFIGURATION
# -------------------------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_RECEIVER_PORTS))
print(f"Listening for data on port {UDP_RECEIVER_PORTS}...")

# -------------------------------------------------
# MAIN
# -------------------------------------------------


def print_seconds():
    threading.Timer(1.0, print_seconds).start()
    print("-" * 80, " ", int(time.time() - start_time), "s")


start_time = time.time()

def main(args=None):
    print_seconds()
    rclpy.init(args=args)
    my_publisher = MyPublisher()

    while True:
        my_publisher.timer_callback()
        print(my_publisher.i)

    my_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
