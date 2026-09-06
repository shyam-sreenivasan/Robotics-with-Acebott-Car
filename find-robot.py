"""Find the car's IP by scanning the local subnet for its command port.

The firmware prints its IP to Serial at boot but never announces itself on
the network, so the only remote signature is TCP 1234 being open (see
commInit() in robot/comm.cpp). We sweep every address on this machine's
subnet and report whoever accepts a connection there.

Run with:  ./venv/bin/python find-robot.py
"""

import ipaddress
import socket
from concurrent.futures import ThreadPoolExecutor

PORT = 1234
TIMEOUT = 0.3   # generous for a LAN; the ESP32 answers in single-digit ms
WORKERS = 256


def local_subnet():
    """Return the /24 this machine sits on, without needing a real route."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # No packets are actually sent; this just picks the outbound interface.
        s.connect(("8.8.8.8", 80))
        my_ip = s.getsockname()[0]
    finally:
        s.close()
    return my_ip, ipaddress.ip_network(f"{my_ip}/24", strict=False)


def probe(ip):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(TIMEOUT)
        if s.connect_ex((str(ip), PORT)) == 0:
            return str(ip)
    return None


def main():
    my_ip, net = local_subnet()
    print(f"This machine: {my_ip}")
    print(f"Scanning {net} for port {PORT} ...")

    hosts = [ip for ip in net.hosts() if str(ip) != my_ip]
    with ThreadPoolExecutor(max_workers=WORKERS) as pool:
        found = [ip for ip in pool.map(probe, hosts) if ip]

    if not found:
        print("\nNo host found with port 1234 open.")
        print("Check that the car is powered on and joined the same network.")
        print("If it is, read the IP off the Arduino Serial Monitor at boot.")
        return

    for ip in found:
        print(f"\nFound: {ip}")
    if len(found) == 1:
        print(f'\nSet in roam.py:  ESP_IP = "{found[0]}"')


if __name__ == "__main__":
    main()
