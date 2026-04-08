#!/bin/bash

pids=$(ps aux | grep web.py | grep -v grep | awk '{print $2}')

if [ -z "$pids" ]; then
  echo "No web.py processes found."
else
  echo "Killing the following web.py processes: $pids"
  kill -9 $pids
  echo "All web.py processes have been killed."
fi

pids=$(ps aux | grep ros | grep -v grep | awk '{print $2}')

if [ -z "$pids" ]; then
  echo "No ROS processes found."
else
  echo "Killing the following ROS processes: $pids"
  kill -9 $pids
  echo "All ROS processes have been killed."
fi