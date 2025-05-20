#!/bin/bash


cd "/home/opossum/robot_ws/" || exit 1

session="robot"

tmux has-session -t $session &> /dev/null
if [ $? == 0 ]; then
	tmux kill-session -t $session
fi

#Create panel
tmux new-session -d -s $session -n "main_control" 
tmux split-window -v
tmux split-window -h
tmux select-pane -t 0
tmux split-window -h
tmux split-window -h


# zenoh window
tmux new-window -t $session -n "telemetry"
tmux split-window -h
tmux send-keys -t $session:1.0 'ros2 topic echo /main_robot/command' 'C-m'
tmux send-keys -t $session:1.1 'ros2 topic echo /main_robot/feedback_command' 'C-m'

tmux select-window -t $SESSION_NAME:0

## launch command in each panel
tmux send-keys -t $session:0.0 'ros2 launch opossum_comm comm.launch.py' 'C-m'
tmux send-keys -t $session:0.1 'ros2 launch opossum_localisation localisation.launch.py' 'C-m'
tmux send-keys -t $session:0.2 'ros2 launch opossum_nav nav.launch.py' 'C-m'
tmux send-keys -t $session:0.3 'ros2 launch opossum_bringup ihm.launch.py' 'C-m'
tmux send-keys -t $session:0.4 'ros2 run opossum_action_sequencer action_sequencer_node.py' 'C-m'
#
tmux attach-session -t $session
