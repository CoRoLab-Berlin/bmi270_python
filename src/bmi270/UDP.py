import numpy as np
import socket
import struct

class UDP:
    def __init__(self, receiver_ip, receiver_port, sender_ip, sender_port) -> None:
        self.receiver_address = (receiver_ip, receiver_port)
        self.sender_address = (sender_ip, sender_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.sender_address)

    def set_receiver_address(self, ip: str, port: int) -> None:
        self.receiver_address = (ip, port)

    def set_sender_address(self, ip: str, port: int) -> None:
        self.sender_address = (ip, port)
        self.sock.bind(self.sender_address)

    def send_data(self, data) -> None:
        self.sock.sendto(data, self.receiver_address)

    def pack_data(self, *args) -> bytes:
        packed_data = bytearray()
        for arg in args:
            arg_32bit = arg.astype(np.int32)
            packed_data += struct.pack('<3i', *arg_32bit)
        return bytes(packed_data)