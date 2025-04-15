import socket
import time
import argparse
import math
import sys
import threading
from collections import defaultdict
# from geopy.distance import geodesic

#   set threads for receiving/sending messages   
#   get_local_ip will not be used on hardware once static ips are set (not needed now, just change to 127.0.0.1)
#   make drones start in different locations -> update locations.txt and sim_vehicle.py for running
#       * ping for each other by awaiting message -> start/bind sockets (2 ports for leader send/follower recv and follower send/leader recv)
#       * both drones go to set target location at 5 m/s  (may have to move this to mavlink to set velocity)  
#       * avoid collision
#       * land when target location reached
#   set mode to loiter after arm

from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

MIN_SAFE_DISTANCE = 20  # in meters need to convert
SOCKET_PORT = 5005
IP_SUBNET = "127.0.0.255" # CHANGE THIS TO "111.111.1.255"
# Define the start and target positions
START_LOCATION = None
TARGET_OFFSET = 0.00028  # Approx. 30m
FLIGHT_ALTITUDE = 10
SAFETY_DISTANCE = 100
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
shared_telemetry = defaultdict(lambda: {"System ID": None, "Latitude": None, "Longitude": None, "Altitude": None, "Vx": None, "Vy": None, "Vz": None})
own_telemetry = defaultdict(lambda: {"System ID": None, "Latitude": None, "Longitude": None, "Altitude": None, "Vx": None, "Vy": None, "Vz": None})
# lock = threading.Lock()
run = True



def takeoff(target_altitude):
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
    time.sleep(5)

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
target_ip = str(IP_SUBNET)
# vehicle.airspeed = 5 # sets default airspeed to 5 m/s

# start socket
# udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# # udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST,1)
# udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT,1)
# udp_socket.bind(('', SOCKET_PORT))
import struct

# MULTICAST_GROUP = "224.1.1.1"  # Any valid multicast IP
# broadcast ip -> 198.
BROADCAST_IP = '127.255.255.255'  # For local test; change to your LAN broadcast IP for real drones
# LOCAL_IFACE = '127.0.0.1'
# SOCKET_PORT = 5005

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)  # ← important on Linux for same-port multicasting
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # ← important on Linux for same-port multicasting


udp_socket.bind(('', SOCKET_PORT))  # '' = all interfaces

# # Join multicast group on the loopback interface (for local testing)
# mreq = struct.pack("4s4s", socket.inet_aton(MULTICAST_GROUP), socket.inet_aton(LOCAL_IFACE))
# udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# # # Optional: allow receiving own messages (for local testing only)
# udp_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)



# connect to mavlink udp sockets
if sys_id == 1:
    print('Trying to connect to 14461')
    master = mavutil.mavlink_connection(f'udp:127.0.0.1:14561')
    master.wait_heartbeat()
elif sys_id == 2:
    print('Trying to connect to 14462')
    master = mavutil.mavlink_connection(f'udp:127.0.0.1:14562')
    master.wait_heartbeat()
    

def send_message():
    while run:
            try:
                # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                # udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST,1)
                # with lock:
                master.wait_heartbeat() # get new message
                msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
                if msg:
                    # Update shared telemetry directly using system_id
                    # shared_telemetry[msg.target_system]["System ID"] = msg.target_system
                    own_telemetry[sys_id]["System ID"] = sys_id
                    own_telemetry[sys_id]["Latitude"] = msg.lat/1e7
                    own_telemetry[sys_id]["Longitude"] = msg.lon/1e7
                    own_telemetry[sys_id]["Altitude"] = msg.alt*0.00328084
                    own_telemetry[sys_id]["Vx"] = msg.vx/100.0
                    own_telemetry[sys_id]["Vy"] = msg.vy/100.0
                    own_telemetry[sys_id]["Vz"] = msg.vz/100.0
                    # broadcast message over entire subnet
                    message = f"{own_telemetry[sys_id]['System ID']},{own_telemetry[sys_id]['Latitude']},{own_telemetry[sys_id]['Longitude']},{own_telemetry[sys_id]['Altitude']},{own_telemetry[sys_id]['Vx']},{own_telemetry[sys_id]['Vy']},{own_telemetry[sys_id]['Vz']}"
                    print(f"Sending: {message} to {IP_SUBNET}:{SOCKET_PORT}")
                    # udp_socket.sendto(message.encode(), (IP_SUBNET, SOCKET_PORT))
                    udp_socket.sendto(message.encode(), (BROADCAST_IP, SOCKET_PORT))

            
            except Exception as e:
                print(f"Error sending data: {e}")
            # finally: 
            #     udp_socket.close()
            time.sleep(1)  # Adjust frequency as required
first_time = True
set_lon = 0
set_lat = 0
set_alt = 0
def receive_message():
    while run:
        # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # udp_socket.bind(('', SOCKET_PORT))

        
        # print("Working")
        try:
            global set_lon
            global set_lat
            global set_alt
            global first_time
            # with lock:
            data, addr = udp_socket.recvfrom(1024)
            # print(f"Received from IP: {addr[0]}, Port: {addr[1]}")
            # time.sleep(1)
            if data:
                
                # Decode data once and split
                parts = data.decode().split(",")
                # Validate the number of parts
                if len(parts) == 7:
                    system_id = int(parts[0])  # The system ID
                    # skip message if broadcasted from own sys_id
                    if (system_id == sys_id):
                        continue
                    
                    lat, lon, alt, vx, vy, vz = map(float, parts[1:])
                    # Update shared telemetry directly using system_id
                    shared_telemetry[system_id]["System ID"] = system_id
                    shared_telemetry[system_id]["Latitude"] = lat
                    shared_telemetry[system_id]["Longitude"] = lon
                    shared_telemetry[system_id]["Altitude"] = alt
                    shared_telemetry[system_id]["Vx"] = vx
                    shared_telemetry[system_id]["Vy"] = vy
                    shared_telemetry[system_id]["Vz"] = vz
                    # other_pred = predict_location(lat,lon,alt,vx,vy,vz)
                    if ((sys_id == 1) & (first_time == True) & ((alt) > 30)):
                        target_location = LocationGlobalRelative(lat+TARGET_OFFSET, lon, (alt/3.2804))
                        vehicle.simple_goto(target_location)
                        set_lat = (lat+TARGET_OFFSET)*1e7
                        set_lon = lon*1e7
                        set_alt = alt/3.2804
                        vehicle.airspeed = 5 # sets default airspeed to 5 m/s
                        first_time = False
                    if ((sys_id == 2) & (first_time == True) & ((alt) > 30)):
                        target_location = LocationGlobalRelative(lat+TARGET_OFFSET, lon, (alt/3.2804))
                        vehicle.simple_goto(target_location)
                        set_lat = (lat+TARGET_OFFSET)*1e7
                        set_lon = lon*1e7
                        set_alt = alt/3.2804
                        vehicle.airspeed = 5 # sets default airspeed to 5 m/s
                        first_time = False
                    # own_pred = predict_location(shared_telemetry[sys_id]["Latitude"],
                    # shared_telemetry[sys_id]["Longitude"],
                    # shared_telemetry[sys_id]["Altitude"],
                    # shared_telemetry[sys_id]["Vx"],
                    # shared_telemetry[sys_id]["Vy"],
                    # shared_telemetry[sys_id]["Vz"])
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
        time.sleep(1)



first = True
def haversine(lat1, lon1, lat2, lon2):
    """Calculate great-circle distance between two GPS points in meters"""
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def run_collision_avoidance_demo():
    # connect to socket
    # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # udp_socket.bind(("0.0.0.0", target_port))  # Bind to the receiving port
    global first
    while first:
        global START_LOCATION
        takeoff(FLIGHT_ALTITUDE)
        first = False
        
# Now that the drone is armed and taken off, start the threads for sending and receiving messages
    send_thread = threading.Thread(target=send_message, daemon=True)
    receive_thread = threading.Thread(target=receive_message, daemon=True)
    
    receive_thread.start()  # Start the receive thread 
    send_thread.start()  # Start the send thread
    
    
        
    
    flag = True
    set_back = True
    # Now, you can continue running your main loop without blocking
    while run:
        global set_lon
        global set_lat
        global set_alt
        global first_time
        if first_time != True:
            own_lat = own_telemetry[sys_id]["Latitude"]
            own_lon = own_telemetry[sys_id]["Longitude"]
            own_alt = own_telemetry[sys_id]["Altitude"]
            for system_id, data in shared_telemetry.items():
                distance = haversine(own_lat,own_lon,data["Latitude"],data["Longitude"])
                print(f"Distance = {distance}")
                if distance < SAFETY_DISTANCE:
                # new_alt = 
                    print(f"[Warning] Drone {system_id} within {distance:.1f}m!")
                    if sys_id > system_id:
                        print("change alt")
                    # vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt + 5))
                        if flag:
                            vehicle.simple_goto(LocationGlobal(set_lat/1e7, set_lon/1e7, set_alt+10))
                            flag = False
                elif (distance > SAFETY_DISTANCE) & (flag == False) & (set_back == True):
                    vehicle.simple_goto(LocationGlobal(set_lat/1e7, set_lon/1e7, set_alt))
                    set_back = False

        time.sleep(1)  # Main loop continues to run, performing any additional tasks

# Main execution loop
try:
    run_collision_avoidance_demo()  # Start the collision avoidance demo
except KeyboardInterrupt:
    print("Shutdown Received")
    run = False
    time.sleep(1)
    
    
    
# def predict_location(lat,lon,alt,vx,vy,vz):
    
#     # dx = shared_telemetry[system_id]["Vx"] * 5
#     # dy = shared_telemetry[system_id]["Vy"] * 5
#     # dz = shared_telemetry[system_id]["Vz"] * 5
#     dx = vx * 5
#     dy = vy * 5
#     dz = vz * 5

#     dlat = dx / 111000
#     dlon = dy / (111000 * math.cos(math.radians(lat)))
#     dalt = dz
    
#     return lat+dlat,dlon+lon,alt+dalt



