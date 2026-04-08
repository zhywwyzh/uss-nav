#!/bin/bash

node_output_file="/home/nv/GNF_WS/log/fy_node_output_${timestamp}.txt"
webpy_output_file="/home/nv/GNF_WS/log/fy_webpy_output_${timestamp}.txt"

tmux new -d -s timesync -n window0;
tmux send -t timesync:window0.0 "bash /home/nv/GNF_WS/src/script/timesync.sh" ENTER;
sleep 8;
tmux kill-session -t timesync;

tmux new -d -s node -n window0;
tmux split-window;
tmux send -t node:window0.0 "node 2>&1 | tee ${node_output_file}" ENTER;
sleep 1;
tmux send -t node:window0.1 "python3 /home/nv/GNF_WS/src/network/web.py" ENTER;
