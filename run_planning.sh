#!/bin/bash

#run ros demo
tmux new-session -s "NewPanel" -d -n "local"
#creat 4 panes in local-window
tmux split-window -v
tmux select-pane -U
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -L
tmux split-window -h
tmux select-pane -L
tmux split-window -h
tmux select-pane -D
tmux split-window -h
tmux select-pane -R
tmux split-window -h
tmux select-pane -L
tmux split-window -h
tmux select-pane -L
tmux split-window -h

#send all commands to each pane

### 


tmux send -t 0 "source /opt/ros/melodic/setup.bash" Enter
tmux send -t 0 "roscore" Enter
sleep 5 #have to sleep 5,to avoid run_id do not match



### 
tmux send -t 1 "cd ~/datang_ws2" Enter
tmux send -t 1 "source devel/setup.sh" Enter
tmux send -t 1 "roslaunch ebot_bringup bringup.launch" Enter
#sleep 1
sleep 1

### 
tmux send -t 2 "cd ~/datang_ws2" Enter
tmux send -t 2 "source devel/setup.bash" Enter
tmux send -t 2 "roslaunch ebot_bringup slam.launch" Enter
sleep 1
tmux send -t 3 "cd ~/datang_ws2" Enter
tmux send -t 3 "source devel/setup.sh" Enter
tmux send -t 3 "roslaunch ebot_bringup costmap.launch" Enter
sleep 1
tmux send -t 4 "cd ~/catkin_realtime" Enter
tmux send -t 4 "source devel/setup.bash" Enter
tmux send -t 4 "roslaunch min_collision_risk_planner ebot_global_planning.launch"  Enter

sleep 1
tmux send -t 5 "cd ~/catkin_realtime" Enter
tmux send -t 5 "source devel/setup.bash" Enter
tmux send -t 5 "roslaunch  ebot_bringup local_planner.launch"  Enter
sleep 1

tmux send -t 6 "cd ~/datang_ws2" Enter
tmux send -t 6 "source devel/setup.bash" Enter
tmux send -t 6 "roslaunch astra_camera gemini.launch"  Enter
sleep 1
tmux send -t 7 "cd ~/datang_ws2" Enter
tmux send -t 7 "source devel/setup.bash" Enter
tmux send -t 7 "roslaunch ultrasonic_sensors ultrasonic_sensors.launch"  Enter

sleep 1
tmux send -t 8 "cd ~/datang_ws2" Enter
tmux send -t 8 "source devel/setup.bash" Enter
tmux send -t 8 "roslaunch ebot_bringup pid_control.launch"
sleep 1

tmux send -t



tmux -2 attach-session -d
