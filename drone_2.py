from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import json
import argparse
import threading

# Argument parser to check if the drone should avoid
parser = argparse.ArgumentParser(description="Drone Collision Avoidance System")
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument("--avoid", action="store_true", help="Enable collision avoidance")
parser.add_argument("--role", choices=["leader", "follower"], required=True, help="Choose role: 'leader' or 'follower'")
args = parser.parse_args()

# Connect to the vehicle
# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
# vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
sys_id = vehicle._master.source_system  # System ID of the drone

# Setup UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Define the start and target positions
START_LOCATION = None
TARGET_OFFSET = 0.00028  # Approx. 20m
FLIGHT_ALTITUDE = 10

def start_listener():
    """Starts the UDP listener in a separate thread."""
    listener_thread = threading.Thread(target=receive_status, daemon=True)
    listener_thread.start()

def send_status(status):
    """Broadcasts the drone's status."""
    message = json.dumps({"sysid": sys_id, "status": status})
    sock.sendto(message.encode(), ('255.255.255.255', 5006))  # Broadcast to all devices

def receive_status():
    """Waits for Drone 1's 'returning' signal before Drone 2 starts flying."""
    sock.bind(('', 5006))  # Listen for messages on port 5006
    sock.settimeout(2)  # Avoid blocking forever

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = json.loads(data.decode())

            if message.get("status") == "returning":
                print("Drone 1 is returning. Drone 2 will now start flight.")
                return
        except socket.timeout:
            continue  # Keep listening

def send_position():
    """Broadcasts the drone's system ID, position, and velocity for avoidance."""
    data = {
        "sysid": sys_id,
        "lat": vehicle.location.global_frame.lat,
        "lon": vehicle.location.global_frame.lon,
        "alt": vehicle.location.global_frame.alt,
        "vx": vehicle.velocity[0],
        "vy": vehicle.velocity[1],
        "vz": vehicle.velocity[2]
    }
    message = json.dumps(data)
    sock.sendto(message.encode(), ('<broadcast>', 5005))  # Broadcast to port 5005


def receive_positions():
    """Receives position data and adjusts altitude if necessary."""
    sock.bind(('', 5005))
    
    while True:
        data, addr = sock.recvfrom(1024)
        info = json.loads(data.decode())

        if info["sysid"] == sys_id:  # Ignore self messages
            continue

        # Calculate distance
        lat_diff = abs(vehicle.location.global_frame.lat - info["lat"])
        lon_diff = abs(vehicle.location.global_frame.lon - info["lon"])
        alt_diff = abs(vehicle.location.global_frame.alt - info["alt"])

        if lat_diff < 0.0001 and lon_diff < 0.0001 and alt_diff < 2:  # Collision threshold
            print(f"Potential collision detected with Drone {info['sysid']}!")
            if args.avoid:
                adjust_altitude()


def adjust_altitude():
    """Increase altitude by 5m to avoid collision."""
    new_alt = vehicle.location.global_frame.alt + 5
    print(f"Adjusting altitude to {new_alt} meters to avoid collision.")
    vehicle.simple_goto(LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, new_alt))


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


def fly_to(target_location):
    """Commands the drone to fly to a specific location."""
    vehicle.simple_goto(target_location)


def run_leader():
    """Executes flight logic for the leader drone."""
    global START_LOCATION
    arm_and_takeoff(FLIGHT_ALTITUDE)
    START_LOCATION = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(START_LOCATION.lat + TARGET_OFFSET, START_LOCATION.lon, FLIGHT_ALTITUDE)

    print("Leader flying out 20 meters...")
    fly_to(target_location)
    time.sleep(5)

    print("Leader returning to start...")
    send_status("returning")
    time.sleep(10)
    fly_to(START_LOCATION)

    time.sleep(10)
    print("Leader landing...")
    vehicle.mode = VehicleMode("LAND")


def run_follower():
    """Executes flight logic for the follower drone."""
    global START_LOCATION
    print("Waiting for leader to start returning...")
    receive_status()

    arm_and_takeoff(FLIGHT_ALTITUDE)
    START_LOCATION = vehicle.location.global_relative_frame
    target_location = LocationGlobalRelative(START_LOCATION.lat + TARGET_OFFSET, START_LOCATION.lon, FLIGHT_ALTITUDE)

    print("Follower flying out 20 meters...")
    fly_to(target_location)
    time.sleep(10)

    print("Follower returning to start...")
    fly_to(START_LOCATION)

    time.sleep(10)
    print("Follower landing...")
    vehicle.mode = VehicleMode("LAND")


def main():
    if args.role == "leader":
        run_leader()
    elif args.role == "follower":
        if args.avoid:
            print("Starting collision avoidance monitoring...")
            receive_positions()
        start_listener()
        run_follower()


if __name__ == "__main__":
    main()
