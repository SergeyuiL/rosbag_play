#!/bin/bash

# Create a new tmux session
session_name="get_rosbag_data_$(date +%s)"
tmux new-session -d -s $session_name

tmux selectp -t 0    # select the first (0) pane
tmux splitw -h -p 50 # split it into two halves

tmux select-pane -t 0
tmux send-keys "roscore" Enter

tmux select-pane -t 1
tmux send-keys "rm -rf data/" Enter
tmux send-keys "sleep 2" Enter
tmux send-keys "python3 scripts/getdata.py" Enter

tmux -2 attach-session -t $session_name