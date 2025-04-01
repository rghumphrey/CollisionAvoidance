#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
usage: python square_off.py --connect <*connection_string>
This script connects to the drone and waits until armed. When armed it will takeoff
to 3m altitude, then navigate a 10x10 meter square. At each corner of the square the drone
will wait for 5 seconds.
"""

from __future__ import print_function

import math
import time
import socket
import sys, os
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))
import mavlink
import json


# Total distance to travel initially
TOT_DIST = 20
# Size of square in meters
SQUARE_SIZE = 5
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 10
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False

# socket globals:
# Socket server port
SOCKET_PORT = 8090
# Buffer size for receiving message
BUFFER_SIZE = 2048
# Server shutdown message
SERVER_SHUTDOWN_MSG = "end of transmission"
# Server timeout
SERVER_TIMEOUT = 60 # 60 seconds


class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)


def start_socket():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, SOCKET_PORT))
        s.listen(1)
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(BUFFER_SIZE)
                if not data:
                    break
                conn.sendall(data)

def send_data():
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
    sock.sendto(message.encode(), ('<broadcast>', 5005))  # Broadcast on port 5005
    print(sending data)

# def receive_data():
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.bind(('', 5005))
#     while True:
#         data, addr = sock.recvfrom(1024)
#         info = json.loads(data.decode())
#         print(f"Received from {addr}: {info}")
#         # Implement altitude change logic based on proximity

def adjust_altitude():
    """Increase altitude to avoid collision."""
    new_alt = vehicle.location.global_frame.alt + 5  # Increase by 5 meters
    print(f"Adjusting altitude to {new_alt} meters")
    vehicle.simple_goto(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, new_alt)


def receive_data():
    """Receives messages, checks for collision risk, and adjusts altitude if necessary."""
    data, addr = sock.recvfrom(1024)
    info = json.loads(data.decode())

    # Ignore messages from itself
    if info["sysid"] == sys_id:
        return

    # Calculate distance (simple approximation)
    lat_diff = abs(vehicle.location.global_frame.lat - info["lat"])
    lon_diff = abs(vehicle.location.global_frame.lon - info["lon"])
    alt_diff = abs(vehicle.location.global_frame.alt - info["alt"])

    # Define a risk threshold
    if lat_diff < 0.0001 and lon_diff < 0.0001 and alt_diff < 2:  # ~10m distance
        print(f"Potential collision detected with Drone {info['sysid']}!")
        if args.avoid:
            adjust_altitude()
    else:
        vehicle.simple_goto(targetLocation)
    return



def set_rtl(value):
    f = fifo()
    print("Creating MAVLink message...")
    # create a mavlink instance, which will do IO on file object 'f'
    mav = mavlink.MAVLink(f)
    # set the WP_RADIUS parameter on the MAV at the end of the link
    mav.param_set_send(0, 0, b"RTL_ALTITUDE", value, mavlink.MAV_PARAM_TYPE_REAL32)

    # alternatively, produce a MAVLink_param_set object 
    # this can be sent via your own transport if you like
    #m = mav.param_set_encode(0, 0, b"WP_RADIUS", 101, mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(3)


def get_dnorth_deast(distance, angle):
    """
    Returns the north and east values for a total distance and a given angle.
    """
    angle_radians = math.radians(angle-90)  # Convert degrees to radians
    north = distance * math.sin(-angle_radians)  # Vertical component
    east = distance * math.cos(angle_radians)  # Horizontal component
    return north, east

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

def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, altitude)

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # delay to wait until yaw of copter is at desired yaw angle
    time.sleep(3)

# Set up option parsing to get connection string and mission plan file
import argparse
parser = argparse.ArgumentParser(description='Drone collision avoidance system.')
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument('--avoid', action="store_true", help="Set this flag if the drone should avoid collisions")
args = parser.parse_args()

# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

print('Succesfully connected to vehicle')

"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")


sys_id = vehicle._master.source_system  # Extract the system ID
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

print(sys_id = {sys_id:.0f})

# Wait for pilot before proceeding
print('Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('Armed...')
vehicle.mode = VehicleMode("GUIDED")

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:

    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)
    
    # Takeoff to short altitude
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(0.5)
    # yaw north
    condition_yaw(0)

# HARD CODED PORTS: FIX THIS
# if (args.avoid):
#     port = 


print(f"Going North 30m")
# Go 20 meters north

# Continuously send data every second
currentLocation=vehicle.location.global_relative_frame
targetLocation=get_location_metres(currentLocation, 30, 0, TARGET_ALTITUDE)
vehicle.simple_goto(targetLocation)

while distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT:
    if args.avoid:
        receive_data()
        time.sleep(1)
    else:
        send_data()
        time.sleep(1)
time.sleep(10)
        

# start the return journey
print(f"Going South 30m")
currentLocation=vehicle.location.global_relative_frame
targetLocation=get_location_metres(currentLocation, -30, 0, TARGET_ALTITUDE)
vehicle.simple_goto(targetLocation)

while distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT:
    if args.avoid:
        receive_data()
    else:
        send_data()
        time.sleep(1)
time.sleep(10)


print('Returning to start location in RTL mode')
# Return to start
vehicle.mode = VehicleMode("RTL")
# #set_rtl(-1)

# msg = vehicle.message_factory.param_set_send(
#     0,0, b"RTL_ALTITUDE", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT8
# )
# # MAV_PARAM_TYPE_INT8
# vehicle.send_mavlink(msg)


# msg = vehicle.message_factory.command_long_encode(
#     0, 0,
#     mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
#     0,0,0,0,0,0,0,0
# )
# vehicle.send_mavlink(msg)

time.sleep(20)

'''
print('Landing')
if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
    # Land Copter
    vehicle.mode = VehicleMode("LAND")
'''
if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
    # disarm Rover
    vehicle.armed = False

# Stay connected to vehicle until landed and disarmed
while vehicle.armed:
    time.sleep(1)

print("Done!")

# Close vehicle object before exiting script
vehicle.close()
