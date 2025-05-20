#!/bin/bash


cd "/home/greg/" || exit 1

session="supervisor"

tmux has-session -t $session &> /dev/null
if [ $? == 0 ]; then
	tmux kill-session -t $session
fi

#Create panel
tmux new-session -d -s $session -n "main" 
tmux split-window -v
tmux split-window -h
tmux select-pane -t 0
tmux split-window -h

# # zenoh window
# tmux new-window -t $session -n "telemetry"
# tmux split-window -h
# tmux send-keys -t $session:1.0 'ros2 topic echo /main_robot/command' 'C-m'
# tmux send-keys -t $session:1.1 'ros2 topic echo /main_robot/feedback_command' 'C-m'

# tmux select-window -t $SESSION_NAME:0

## launch command in each panel
tmux send-keys -t $session:0.0 'src_opossum && robot_domain && ros2 run plotjuggler plotjuggler' 'C-m'
tmux send-keys -t $session:0.1 'src_opossum && robot_domain && rviz2' 'C-m'
tmux send-keys -t $session:0.2 'src_opossum && robot_domain && ros2 launch opossum_dev_gui dev_gui.launch.py' 'C-m'
tmux send-keys -t $session:0.3 ''
#
tmux attach-session -t $session
