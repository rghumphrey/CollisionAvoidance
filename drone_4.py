import socket
import time
import argparse
from dronekit import connect


# def get_local_ip():
#     """Retrieves the local machine's IP address."""
#     hostname = socket.gethostname()
#     ip_address = socket.gethostbyname(hostname)
#     return ip_address

import netifaces

def get_local_ip():
    """Retrieve the primary IP address of the active interface."""
    for iface in netifaces.interfaces():
        # Look for active interfaces with an IPv4 address
        iface_details = netifaces.ifaddresses(iface).get(netifaces.AF_INET)
        if iface_details:
            ip_address = iface_details[0].get('addr')
            if ip_address != "127.0.0.1":  # Exclude loopback
                return ip_address
    return "No external IP address found"



vehicle = connect('127.0.0.1:14552', wait_ready=True)
msg = vehicle._master.wait_heartbeat() 
sys_id = msg.get_srcSystem()
print(f"System ID: {sys_id}")


print(f"Target IP: {vehicle._master.address}")

machine_ip = get_local_ip()
print(f"The machine's IP address is: {machine_ip}")

