import socket
import threading
import time

# -------------------------------------------------
# DEFINES
# -------------------------------------------------

UDP_IP              = "0.0.0.0"
UDP_RECEIVER_PORTS  = [33771, 33772]
UDP_SENDER_PORTS    = [33881, 33882]


# -------------------------------------------------
# INITIALIZATION
# -------------------------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


socks = []
for port in UDP_RECEIVER_PORTS:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, port))
    print(f"Listening for data on port {port}...")
    socks.append(sock)

# -------------------------------------------------
# FUNCTIONS
# -------------------------------------------------

def print_seconds():
    threading.Timer(1.0, print_seconds).start()
    print("-" * 130, " ", int(time.time() - start_time), "s")

def receive_data():
    threading.Timer(0.02, receive_data).start()

    for sock in socks:
        data, address = sock.recvfrom(1024)
        
        if address[1] == UDP_SENDER_PORTS[0]:
            print('Received from BMI270_1:', data.decode(), end="\t")
        elif address[1] == UDP_SENDER_PORTS[1]:
            print('Received from BMI270_2:', data.decode())
        else:
            print(f'Received from unknown device: {address[1]}', data.decode())




# -------------------------------------------------
# MAIN
# -------------------------------------------------

print("Setup complete")
start_time = time.time()

def main():
    print_seconds()
    receive_data()

if __name__ == "__main__":
    main()