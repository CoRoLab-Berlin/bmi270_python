import numpy as np
import socket
import struct
import threading
import time

from src.bmi270.definitions import *

# -------------------------------------------------
# DEFINES
# -------------------------------------------------

UDP_IP = '0.0.0.0'
UDP_RECEIVER_PORTS = 33771
UDP_SENDER_PORT = 33881


# -------------------------------------------------
# INITIALIZATION
# -------------------------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_RECEIVER_PORTS))
print(f"Listening for data on port {UDP_RECEIVER_PORTS}...")

# -------------------------------------------------
# FUNCTIONS
# -------------------------------------------------


def print_seconds():
    threading.Timer(1.0, print_seconds).start()
    print("-" * 80, " ", int(time.time() - start_time), "s")

def convert_data(data):
    
    # Convert accerleration data to SI units
    scaling1 = data[1] / 2*GRAVITY
    scaling2 = data[2] / 2
    scaling3 = data[3] / 

    pass

# -------------------------------------------------
# MAIN
# -------------------------------------------------

start_time = time.time()

def main():
    print_seconds()

    BMI270_1 = []
    BMI270_2 = []

    try:
        while True:
            received_data, address = sock.recvfrom(1024)
            unpacked_data = struct.unpack('<i3i3ii3i3i', received_data)
            print(unpacked_data)
            unpacked_data = convert_data(unpacked_data)
            BMI270_1.append(unpacked_data[:7])
            BMI270_2.append(unpacked_data[7:])
    except KeyboardInterrupt:
        print("--- STOPPED RECEIVING DATA ---")

    BMI270_1 = np.array(BMI270_1)
    BMI270_2 = np.array(BMI270_2)
    print("BMI270_1:", BMI270_1)
    print("BMI270_2:", BMI270_2)


if __name__ == "__main__":
    main()
