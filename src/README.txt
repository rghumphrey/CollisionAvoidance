To run the collision avoidance script, ensure that you are connected to the network the Raspberry Pi's start. Then, ssh into the first drone with:

ssh drone1@IP_ADDRESS (where the IP address is the drone's IP)

Launch mavproxy with this command: 

mavproxy.py --master=/dev/ttyACM0 --baudrate 57600 --out 127.0.0.1:14551 --out 127.0.0.1:14561 

Then run this to start the script: 

python3 collision_avoidance.py --connect :14551

Repeat this for the second drone. 

To run the scripts for swarm, follow a similar procedure but instead of running python3 collision_avoidance.py, run drone_swarm_master.py on the first drone and drone_swarm_follower.py on the second.
