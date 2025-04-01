#!/bin/bash

#kill all screens:
#killall screen
cd ~/sitl

# start screens
screen -S drone_sitl_1 -d -m
screen -S drone_sitl_2 -d -m
screen -S drone_mission_1 -d -m
screen -S drone_mission_2 -d -m
screen -S QGroundControl -d -m

# start sitl for both drones in each screen:
screen -S drone_sitl_1 -X stuff "sim_vehicle.py -v copter --out="udp:127.0.0.1:14551" --add-param-file=network_params.parm\n"
screen -S drone_sitl_2 -X stuff "sim_vehicle.py -v copter --instance=1 --sysid=2 --out="udp:127.0.0.1:14552" --add-param-file=network_params_2.parm\n"

#launch QGroundControl:
screen -S QGroundControl -X stuff "./QGroundControl.AppImage\n"

#run missions:
#screen -S drone_mission_1 -X stuff "python3 drone_3.py --connect :14551 --role leader"
#screen -S drone_mission_2 -X stuff "python3 drone_3.py --connect :14552 --role follower"

