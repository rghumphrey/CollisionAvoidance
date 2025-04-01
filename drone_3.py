import socket
import time
import argparse
import math
import netifaces

from dronekit import connect, VehicleMode, LocationGlobalRelative

SOCKET_PORT = 5005
# Define the start and target positions
START_LOCATION = None
TARGET_OFFSET = 0.00028  # Approx. 20m
FLIGHT_ALTITUDE = 10
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1

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
    vehicle.armed = True
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

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distanceToWaypoint(coordinates):
    """
    Returns distance between vehicle and specified coordinates
    """
    distance = get_distance_metres(vehicle.location.global_frame, coordinates)
    return distance

def adjust_altitude():
    """Increase altitude by 5m to avoid collision."""
    new_alt = vehicle.location.global_frame.alt + 5
    print(f"Adjusting altitude to {new_alt} meters to avoid collision.")
    vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, new_alt))

    
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

def send_message():
    position = vehicle.location.global_frame
    message = f"{sys_id},{position.lat},{position.lon},{position.alt}"
    print(f"Sending: {message} to {target_ip}:{target_port}")
    udp_socket.sendto(message.encode(), (target_ip, target_port))
    time.sleep(1)  # Adjust frequency as required

def receive_message():
    try:    # PROGRAM GETS STUCK HERE
        data, addr = udp_socket.recvfrom(1024)
        print(f"Received: {data.decode()} from {addr}")
    except socket.timeout:
        print("Socket timed out while waiting for data.")
    except Exception as e:
        print(f"Error receiving data: {e}")
    # data, addr = udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes

    id, lat, lon, alt = map(float, data.decode().split(","))
    return id, lat, lon, alt 
    

def run_leader():
    # start socket and transmissions
    # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # take off and remember location, then fly out
    # currentLocation=vehicle.location.global_relative_frame
    # targetLocation=get_location_metres(currentLocation, 30, 0, TARGET_ALTITUDE)
    # vehicle.simple_goto(targetLocation)

    global START_LOCATION
    arm_and_takeoff(FLIGHT_ALTITUDE)
    START_LOCATION = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(START_LOCATION.lat + TARGET_OFFSET, START_LOCATION.lon, FLIGHT_ALTITUDE)

    # go out
    vehicle.simple_goto(target_location)
    while distanceToWaypoint(target_location) > WAYPOINT_LIMIT:
        send_message()
        time.sleep(0.5)
    
    # wait and launch other drone
    time.sleep(10)
    target_location = START_LOCATION
    # come back
    vehicle.simple_goto(target_location)
    while distanceToWaypoint(target_location) > WAYPOINT_LIMIT:
        send_message()
        time.sleep(0.5)
    
    print("Leader landing...")
    vehicle.mode = VehicleMode("LAND")



def run_follower():
    # connect to socket
    # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("0.0.0.0", target_port))  # Bind to the receiving port

    # take off and remember location, then fly out
    global START_LOCATION
    arm_and_takeoff(FLIGHT_ALTITUDE)
    START_LOCATION = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(START_LOCATION.lat + TARGET_OFFSET, START_LOCATION.lon, FLIGHT_ALTITUDE)
    
    # go out
    vehicle.simple_goto(target_location)
    while distanceToWaypoint(target_location) > WAYPOINT_LIMIT:    

        id, lat, lon, alt = receive_message()
        if id == sys_id: 
            continue
        print(f"Received position: Lat={lat}, Lon={lon}, Alt={alt}")
        # Collision avoidance or navigation logic goes here

        # Calculate distance
        lat_diff = abs(vehicle.location.global_frame.lat - lat)
        lon_diff = abs(vehicle.location.global_frame.lon - lon)
        alt_diff = abs(vehicle.location.global_frame.alt - alt)

        if lat_diff < 0.0001 and lon_diff < 0.0001 and alt_diff < 2:  # Collision threshold
            print(f"Potential collision detected with Drone {id}!")
            adjust_altitude()
        else:
            vehicle.simple_goto(target_location)
        
        time.sleep(0.5)

    # wait 
    time.sleep(10)
    target_location = START_LOCATION
    # come back
    vehicle.simple_goto(target_location)
    while distanceToWaypoint(target_location) > WAYPOINT_LIMIT:
        id, lat, lon, alt = receive_message()
        if id == sys_id: 
            continue
        print(f"Received position: Lat={lat}, Lon={lon}, Alt={alt}")
        # Collision avoidance or navigation logic goes here

        # Calculate distance
        lat_diff = abs(vehicle.location.global_frame.lat - lat)
        lon_diff = abs(vehicle.location.global_frame.lon - lon)
        alt_diff = abs(vehicle.location.global_frame.alt - alt)

        if lat_diff < 0.0001 and lon_diff < 0.0001 and alt_diff < 2:  # Collision threshold
            print(f"Potential collision detected with Drone {info['sysid']}!")
            adjust_altitude()
        else:
            vehicle.simple_goto(target_location)

        time.sleep(0.5)
    
    print("Leader landing...")
    vehicle.mode = VehicleMode("LAND")


udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

if args.role == "leader":
    run_leader()
elif args.role == "follower":
    run_follower()


