import socket
import time
import argparse
import netifaces

from dronekit import connect

SOCKET_PORT = 5005

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


# Argument parser to check if the drone should avoid
parser = argparse.ArgumentParser(description="Drone Collision Avoidance System")
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument("--avoid", action="store_true", help="Enable collision avoidance")
parser.add_argument("--role", choices=["leader", "follower"], required=True, help="Choose role: 'leader' or 'follower'")
args = parser.parse_args()

# Connect to the vehicle
connection_string = args.connect
if not connection_string:
    sys.exit('Please specify connection string')
# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


# vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
msg = vehicle._master.wait_heartbeat() 
sys_id = msg.get_srcSystem()
print(f"sys id = {sys_id:.0f}")
target_ip = str(get_local_ip())
target_port = SOCKET_PORT
print(f"The machine's IP address is: {target_ip}")


def run_leader():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("send:")
    while True:
        position = vehicle.location.global_frame
        message = f"{sys_id},{position.lat},{position.lon},{position.alt}"
        print(f"Sending: {message} to {target_ip}:{target_port}")
        udp_socket.sendto(message.encode(), (target_ip, target_port))
        time.sleep(1)  # Adjust frequency as required


def run_follower():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("0.0.0.0", target_port))  # Bind to the receiving port
    print("recv:")
    while True:
        # print("check1")
        try:    # PROGRAM GETS STUCK HERE
            data, addr = udp_socket.recvfrom(1024)
            print(f"Received: {data.decode()} from {addr}")
        except socket.timeout:
            print("Socket timed out while waiting for data.")
        except Exception as e:
            print(f"Error receiving data: {e}")
        # data, addr = udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        sys_id, lat, lon, alt = map(float, data.decode().split(","))
        print(f"Received position: Lat={lat}, Lon={lon}, Alt={alt}")
        # Collision avoidance or navigation logic goes here



if args.role == "leader":
    run_leader()
elif args.role == "follower":
    run_follower()


