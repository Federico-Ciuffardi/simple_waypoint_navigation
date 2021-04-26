#! /usr/bin/env bash

rostopic pub -1 /p3dx_0/waypoint geometry_msgs/Point   '{x: -5.0, y: 0.0, z: 0.0}'
sleep 6                                                                   
while true ; do                                                           
  rostopic pub -1 /p3dx_0/waypoint geometry_msgs/Point '{x: -5.0, y: 5.0, z: 0.0}'
  sleep 12                                                               
  rostopic pub -1 /p3dx_0/waypoint geometry_msgs/Point '{x: -5.0, y: -5.0, z: 0.0}'
  sleep 12                                                                
  rostopic pub -1 /p3dx_0/waypoint geometry_msgs/Point '{x: 5.0, y: -5.0, z: 0.0}'
  sleep 12                                                                
  rostopic pub -1 /p3dx_0/waypoint geometry_msgs/Point '{x: 5.0, y: 5.0, z: 0.0}'
  sleep 12
done
