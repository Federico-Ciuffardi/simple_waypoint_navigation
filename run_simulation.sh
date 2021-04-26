cd ~/catkin_ws 
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 || exit 1
rqt_console & 
rviz -d /home/fede/catkin_ws/src/simple_waypoint_navigation/rviz/rviz.rviz & 
roslaunch simple_waypoint_navigation simple_waypoint_navigation.launch  
