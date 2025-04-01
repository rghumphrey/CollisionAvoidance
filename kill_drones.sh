#!/bin/bash

killall screen

# Get all PIDs of xterm processes
pids=$(ps aux | grep '[x]term' | awk '{print $2}')

# Check if any PIDs were found
if [ -z "$pids" ]; then
    echo "No xterm processes found."
else
    for pid in $pids; do
        kill $pid
    done
fi
