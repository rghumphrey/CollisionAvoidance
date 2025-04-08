import socket
import time
import argparse
import math
import netifaces
import sys
import threading
from collections import defaultdict

#   set threads for receiving/sending messages   
#   get_local_ip will not be used on hardware once static ips are set (not needed now, just change to 127.0.0.1)
#   make drones start in different locations -> update locations.txt and sim_vehicle.py for running
#       * ping for each other by awaiting message -> start/bind sockets (2 ports for leader send/follower recv and follower send/leader recv)
#       * both take off then wait a few seconds until target altitude is reached, then send handshakes  -> sys id dependent for collision avoidance but changed to 1 function rather than both leader/follower
#       * set target locations to each other and condition yaw -> send/recv message with location (lat/lon/alt), call condition yaw
#       * both drones go to set target location at 5 m/s  (may have to move this to mavlink to set velocity)  
#       * avoid collision
#       * land when target location reached
#   set mode to loiter after arm


from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative

MIN_SAFE_DISTANCE = 20  # in meters need to convert
SOCKET_PORT_LEADER_SEND = 5005
SOCKET_PORT_FOLLOWER_SEND = 5006
# Define the start and target positions
START_LOCATION = None
TARGET_OFFSET = 0.00028  # Approx. 20m
FLIGHT_ALTITUDE = 10
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
shared_telemetry = defaultdict(lambda: {"System ID": None, "Latitude": None, "Longitude": None, "Altitude": None, "Heading": None})

lock = threading.Lock()
run = True

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

def arm_and_takeoff(target_altitude):
    """Arms and takes off to the target altitude."""
    while not vehicle.is_armable:
        print("Waiting for drone to be armable...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")

    while not vehicle.armed:
        print("Waiting for drone to arm...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    
# Argument parser to check if the drone should avoid
parser = argparse.ArgumentParser(description="Drone Collision Avoidance System")
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument("--avoid", action="store_true", help="Enable collision avoidance")
# parser.add_argument("--role", choices=["leader", "follower"], required=True, help="Choose role: 'leader' or 'follower'")
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
if sys_id == 1: 
    target_port_leader = SOCKET_PORT_LEADER_SEND   #leader or follower
    target_port_follower = SOCKET_PORT_FOLLOWER_SEND  #leader or follower
elif sys_id == 2:
    target_port_leader = SOCKET_PORT_FOLLOWER_SEND   #leader or follower
    target_port_follower = SOCKET_PORT_LEADER_SEND   #leader or follower
print(f"The machine's IP address is: {target_ip}")

if sys_id == 1:
    print('Trying to connect to 14461')
    master = mavutil.mavlink_connection(f'udp:127.0.0.1:14561')
    master.wait_heartbeat()
elif sys_id == 2:
    print('Trying to connect to 14462')
    master = mavutil.mavlink_connection(f'udp:127.0.0.1:14562')
    master.wait_heartbeat()
    
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT,1)
udp_socket.bind(("0.0.0.0", target_port_leader))  # Bind to the receiving port    
udp_socket_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket_receive.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT,1)
# udp_socket_receive.setblocking(False)
udp_socket_receive.settimeout(1)  # Set a longer timeout for testing
udp_socket_receive.bind(("0.0.0.0", target_port_follower))  # Bind to the receiving port
# udp_socket_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# udp_socket_send.bind(("0.0.0.0", target_port_leader))  # Bind to the receiving port

def send_message():
    while run:
            try:
                # with lock:
                    msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
                    if msg:
                        # Update shared telemetry directly using system_id
                        shared_telemetry[sys_id]["Latitude"] = msg.lat/1e7
                        shared_telemetry[sys_id]["Longitude"] = msg.lon/1e7
                        shared_telemetry[sys_id]["Altitude"] = msg.alt*0.00328084
                        shared_telemetry[sys_id]["Heading"] = msg.hdg/100
                        
                        message = f"{sys_id},{shared_telemetry[sys_id]["Latitude"] },{shared_telemetry[sys_id]["Longitude"]},{shared_telemetry[sys_id]["Altitude"]},{shared_telemetry[sys_id]["Heading"]}"
                        global target_port_leader
                        print(f"Sending: {message} to {target_ip}:{target_port_leader}")
                        udp_socket.sendto(message.encode(), (target_ip, target_port_leader))
            except Exception as e:
                print(f"Error sending data: {e}")
            time.sleep(0.5)  # Adjust frequency as required

def receive_message():
    while run:
        try:
            # with lock:
                data, addr = udp_socket_receive.recvfrom(1024)
            

                if data:
                    # Decode data once and split
                    parts = data.decode().split(",")
            
                    # Validate the number of parts
                    if len(parts) == 5:
                        system_id = int(parts[0])  # The system ID
                        lat, lon, alt, head = map(float, parts[1:])
                
                        # Update shared telemetry directly using system_id
                        shared_telemetry[system_id]["Latitude"] = lat
                        shared_telemetry[system_id]["Longitude"] = lon
                        shared_telemetry[system_id]["Altitude"] = alt
                        shared_telemetry[system_id]["Heading"] = head
                        print(f"Updated shared telemetry for system {system_id}: {shared_telemetry[system_id]}")
                    else:
                        print(f"Invalid data format received: {data.decode()}")
                else:
                    print("Received empty data")
        
        except socket.timeout:
            print("Socket timed out while waiting for data.")
        except ValueError:
            print("Error processing data (invalid number format).")
        except Exception as e:
            print(f"Error receiving data: {e}")



# time.sleep(5)
# Create thread

# receive_thread = threading.Thread(target=receive_message, daemon=True)#.start()
# send_thread = threading.Thread(target=send_message, daemon=True)#.start()
# # Start thread

# receive_thread.start()
# send_thread.start()

# # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# udp_socket.bind(("0.0.0.0", target_port))  # Bind to the receiving port
first = True
def run_collision_avoidance_demo():
    # connect to socket
    # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # udp_socket.bind(("0.0.0.0", target_port))  # Bind to the receiving port
    global first
    while first:
        global START_LOCATION
        arm_and_takeoff(FLIGHT_ALTITUDE)
        first = False
        
#     send_thread.run()
#     receive_thread.run()
#     time.sleep(1)
    
# try:
#     while (run):
#         run_collision_avoidance_demo()
# except KeyboardInterrupt:
#     print("Shutdown Received")
#     run = False
#     time.sleep(1)
# Now that the drone is armed and taken off, start the threads for sending and receiving messages
    send_thread = threading.Thread(target=send_message, daemon=True)
    receive_thread = threading.Thread(target=receive_message, daemon=True)
    
    receive_thread.start()  # Start the receive thread 
    send_thread.start()  # Start the send thread
    # receive_thread.start()  # Start the receive thread
    
    # Now, you can continue running your main loop without blocking
    while run:
        time.sleep(1)  # Main loop continues to run, performing any additional tasks

# Main execution loop
try:
    run_collision_avoidance_demo()  # Start the collision avoidance demo
except KeyboardInterrupt:
    print("Shutdown Received")
    run = False
    time.sleep(1)