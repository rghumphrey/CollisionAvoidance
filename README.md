### Collision Avoidance

These files right now:

Drone 1 doesn't work

Drone 2 doesn't work

Drone 3 will have a leader broadcast messages and a follower receive them, you need to specify flags for drone as --role leader or --role follower when launching

Drone 4 was for debugging things

network_param files were an attempt to set the IP of the drones but that wasn't working



**How to Use**

This assumes you have a sitl folder in home directory.

Make sure whatever drone mission file (ex. drone_3.py) is in sitl folder. Run start_drones.sh to launch a bunch of screens, launch two drones in sitl, and start QGroundControl (if QGroundControl is also in sitl folder)
kill_drones.sh will close screens and xterm

In start_drones.sh there is the run commands for the missions commented out. These will show how to launch missions

**Screen Stuff**

to attach to a screen to run a mission, you can use (this will attach to drone mission 1 screen)
```
screen -r drone_mission_1
```
to detatch, use Ctrl+a then Ctrl+d
