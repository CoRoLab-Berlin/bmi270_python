import numpy as np
import socket
import struct
import threading
import time

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



# -------------------------------------------------
# MAIN
# -------------------------------------------------

start_time = time.time()


def main():
    print_seconds()

    while True:
        received_data, address = sock.recvfrom(1024)
        unpacked_data = struct.unpack('<i3i3ii3i3i', received_data)
        print(unpacked_data)


if __name__ == "__main__":
    main()
