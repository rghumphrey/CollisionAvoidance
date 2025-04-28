import socket
import time
import argparse
import math
import sys
import threading
from collections import defaultdict
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

SOCKET_PORT = 5005
IP_SUBNET = "127.0.0.255" # CHANGE THIS TO "10.42.0.255"
START_LOCATION = None
TARGET_OFFSET = 0.00028  # Approx. 30m
FLIGHT_ALTITUDE = 5
SAFETY_DISTANCE = 30
WAYPOINT_LIMIT = 1
shared_telemetry = defaultdict(lambda: {"System ID": None, "Latitude": None, "Longitude": None, "Altitude": None, "Vx": None, "Vy": None, "Vz": None})
own_telemetry = defaultdict(lambda: {"System ID": None, "Latitude": None, "Longitude": None, "Altitude": None, "Vx": None, "Vy": None, "Vz": None})
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
    return None


def takeoff(target_altitude):
    """Arms and takes off to the target altitude."""
    while not vehicle.is_armable:
        print("Waiting for drone to be armable...")
        time.sleep(1)

    
    
   
    while not vehicle.armed:
        print("Waiting for drone to arm...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(2)
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

ip = get_local_ip()
if ip:
    try:
        last_octet = int(ip.split('.')[-1])
        if last_octet == 1:
            sys_id = 1
        else:
           sys_id = 2
    except ValueError:
        print("Invalid IP format.")

msg = vehicle._master.wait_heartbeat()

print(f"sys id = {sys_id:.0f}")
target_ip = str(IP_SUBNET)

# start socket

BROADCAST_IP = '127.255.255.255'  # For local test;  and need change to your broadcast IP for real drones


udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # ← important on Linux for same-port multicasting


udp_socket.bind(('', SOCKET_PORT))  # '' = all interfaces



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
    global run
    while run:
            try:
               
                master.wait_heartbeat() # get new message
                msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
                if msg:
                    own_telemetry[sys_id]["System ID"] = sys_id
                    own_telemetry[sys_id]["Latitude"] = msg.lat/1e7
                    own_telemetry[sys_id]["Longitude"] = msg.lon/1e7
                    own_telemetry[sys_id]["Altitude"] = msg.alt*0.00328084
                    own_telemetry[sys_id]["Vx"] = msg.vx/100.0
                    own_telemetry[sys_id]["Vy"] = msg.vy/100.0
                    own_telemetry[sys_id]["Vz"] = msg.vz/100.0
                    message = f"{own_telemetry[sys_id]['System ID']},{own_telemetry[sys_id]['Latitude']},{own_telemetry[sys_id]['Longitude']},{own_telemetry[sys_id]['Altitude']},{own_telemetry[sys_id]['Vx']},{own_telemetry[sys_id]['Vy']},{own_telemetry[sys_id]['Vz']}"
                    print(f"Sending: {message} to {IP_SUBNET}:{SOCKET_PORT}")
                    udp_socket.sendto(message.encode(), (BROADCAST_IP, SOCKET_PORT))

            
            except Exception as e:
                print(f"Error sending data: {e}")
            
            time.sleep(1) 
first_time = True
set_lon = 0
set_lat = 0
set_alt = 0
def receive_message():
    global run
    while run:
        try:
            global set_lon
            global set_lat
            global set_alt
            global first_time
            data, addr = udp_socket.recvfrom(1024)
            
            if data:
                
                parts = data.decode().split(",")
                if len(parts) == 7:
                    system_id = int(parts[0])  
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
                    if ((sys_id == 1) & (first_time == True) & ((alt) > 15)):
                        target_location = LocationGlobal(lat+TARGET_OFFSET, lon, (alt/3.2804))
                        vehicle.simple_goto(target_location)
                        set_lat = (lat+TARGET_OFFSET)*1e7
                        set_lon = lon*1e7
                        set_alt = alt/3.2804
                        vehicle.airspeed = 1 # sets default airspeed to 1 m/s
                        first_time = False
                    if ((sys_id == 2) & (first_time == True) & ((alt) > 15)):
                        target_location = LocationGlobal(lat+TARGET_OFFSET, lon, (alt/3.2804))
                        vehicle.simple_goto(target_location)
                        set_lat = (lat+TARGET_OFFSET)*1e7
                        set_lon = lon*1e7
                        set_alt = alt/3.2804
                        vehicle.airspeed = 1 # sets default airspeed to 1 m/s
                        first_time = False

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
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def run_collision_avoidance_demo():
    
    global first
    global run
    while first:
        takeoff(FLIGHT_ALTITUDE)
        first = False
        
# Now that the drone is armed and taken off, start the threads for sending and receiving messages
    send_thread = threading.Thread(target=send_message, daemon=True)
    receive_thread = threading.Thread(target=receive_message, daemon=True)
    
    receive_thread.start()  # Start the receive thread 
    send_thread.start()  # Start the send thread
    
    
        
    
    flag = True
    set_back = True
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
                
                    print(f"[Warning] Drone {system_id} within {distance:.1f}m!")
                    if sys_id > system_id:
                        print("change alt")
                    
                        if flag:
                            vehicle.simple_goto(LocationGlobal(set_lat/1e7, set_lon/1e7, set_alt+10))
                            flag = False
                elif (distance > SAFETY_DISTANCE) & (flag == False) & (set_back == True):
                    vehicle.simple_goto(LocationGlobal(set_lat/1e7, set_lon/1e7, set_alt))
                    set_back = False
          

        time.sleep(1)  
    

# Main execution loop
try:
    run_collision_avoidance_demo()  # Start the collision avoidance demo
except KeyboardInterrupt:
    print("Shutdown Received")
    run = False
    time.sleep(1)
    
    




