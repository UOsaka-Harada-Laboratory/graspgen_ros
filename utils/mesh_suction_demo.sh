#!/bin/bash

byobu new-session -d -s mesh_suction_demo
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 2
byobu split-window -v

byobu send-keys -t 0 'xhost + && docker exec -it graspgen_container bash -it -c "meshcat-server"' 'C-m'
byobu send-keys -t 1 'xhost + && docker exec -it graspgen_container bash -it -c "ros2 launch graspgen_tutorials mesh_graspgen_launch.py config:=/ros2_ws/src/graspgen_tutorials/config/example_mesh_suction_generator.yaml"' 'C-m'
sleep 1.
byobu send-keys -t 2 'xhost + && docker exec -it graspgen_container bash -it -c "ros2 service call /generate_grasp std_srvs/srv/Empty"' 'C-m'

byobu attach -t mesh_suction_demo
