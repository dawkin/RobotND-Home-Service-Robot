#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/simple_space_fts.world " & 

sleep 5

# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../../src/map/map.yaml " &

sleep 5

# launch view_navigation for rviz
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 20 

# launch add_markers node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun add_markers add_markers_test " &
