/*
 * To run gmapping you should start gmapping:
 * rosrun gmapping slam_gmapping scan:=lidar_topic _xmax:=30 _xmin:=-30 _ymax:=30 _ymin:=-30
 * _delta:=0.2
 */

#include <cmath>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/ros.h"
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <utility>

#include <math.h>
#include <stdexcept>      // std::out_of_range
#include <queue> 
#include <algorithm>    // std::max

#include "Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define TIME_STEP 32
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9
#define NMOTORS 2

using namespace std;

double min_angular_speed = 0.2;
double max_angular_speed = 1;

double min_linear_speed = 0.2;
double max_linear_speed = 1;

double yaw_tolerance = 0.1;
double distance_tolerance = 0.1;

ros::NodeHandle *n;

ros::Publisher cmdVelPub;

Vector3 position;
geometry_msgs::Quaternion orientation;

/* queue<Vector3> waypoints; */
Vector3* wp;

// motors
void updateSpeed(float linear_speed, float angular_speed) {
  geometry_msgs::Vector3 linear;
  linear.x = linear_speed;
  linear.y = 0;
  linear.z = 0;

  geometry_msgs::Vector3 angular;
  angular.x = 0;
  angular.y = 0;
  angular.z = angular_speed;

  geometry_msgs::Twist vel;
  vel.linear=linear;
  vel.angular=angular;

  cmdVelPub.publish(vel);
}

Vector3 point2Vector3(geometry_msgs::Point p){
  return Vector3(p.x,p.y,p.z);
}

double simplify_angle(double angle){
  fmod(angle,2*M_PI);
  if(angle<0){
    angle += 2*M_PI;
  }
  return angle;
}

void goToWaypoints(){
  /* if(waypoints.size() == 0){ */
  if (wp == NULL){
    updateSpeed(0, 0);
  }else{
    Vector3 waypoint = *wp;
    /* Vector3 waypoint = waypoints.front(); */
    Vector3 position_waypoint_vector =  waypoint - position; 
    float linear_distance = (position_waypoint_vector).length();
    /* ROS_INFO_STREAM_THROTTLE(1,"linear_distance"<<linear_distance); */
    if ( linear_distance < distance_tolerance){
      ROS_INFO_STREAM("waypoint compleated: ("<<waypoint.x<<","<<waypoint.y<<","<<waypoint.z<<")");
      wp = NULL;
      /* waypoints.pop(); */
    }else{
      // get robot_yaw
      tf2::Quaternion q;
      tf2::convert(orientation, q);
      tf2::Matrix3x3 m(q);
      double robot_roll, robot_pitch, robot_yaw;
      m.getRPY(robot_roll, robot_pitch, robot_yaw);

      // get waypoint yaw (yaw of the waypoint_position - robot_position)
      double waypoint_yaw = atan2(position_waypoint_vector.y,position_waypoint_vector.x);
      /* ROS_INFO_STREAM_THROTTLE(1,"waypoint_yaw"<<waypoint_yaw); */
      /* ROS_INFO_STREAM_THROTTLE(1,"robot_yaw"<<robot_yaw); */
      
      // get the angular_distance linear_distance
      /// find the delta between the waypoint_yaw and the robot_yaw
      double angular_distance = simplify_angle(simplify_angle(waypoint_yaw) - simplify_angle(robot_yaw));
      if ( abs(angular_distance - 2*M_PI) < abs(angular_distance) ){
        angular_distance = angular_distance - 2*M_PI;
      }
      /// normailize it
      /* while (abs(angular_distance) > M_PI){ */
      /*   angular_distance -= copysign(M_PI,angular_distance); */
      /* } */

      /* ROS_INFO_STREAM_THROTTLE(1,"angular_distance"<<angular_distance); */
      double angular_speed = angular_distance; 
      if (abs(angular_speed) > max_angular_speed){ // force max value
        angular_speed = copysign(max_angular_speed,angular_distance);
      } else if (abs(angular_speed) < min_angular_speed){ // force min value
        angular_speed = copysign(min_angular_speed,angular_speed);
      }
      /* ROS_INFO_STREAM_THROTTLE(1,"angular_speed"<<angular_distance); */
      if ( abs(angular_distance) > yaw_tolerance ){
        updateSpeed(0, angular_speed);
      } else {
        /* ROS_INFO_STREAM_THROTTLE(1,"linear_distance"<<linear_distance); */
        double linear_speed = linear_distance;
        if (abs(linear_speed) > max_linear_speed){ // force max value
          linear_speed = copysign(max_linear_speed,linear_speed);
        } else if (abs(linear_speed) < min_linear_speed){ // force min value
          linear_speed = copysign(min_linear_speed,linear_speed);
        }
        /* ROS_INFO_STREAM_THROTTLE(1,"linear_speed"<<linear_speed); */
        updateSpeed(linear_speed, angular_speed);
      }
    }    
  }
}

void waypointCallback(const geometry_msgs::Point::ConstPtr &point) {
  wp = new Vector3(point->x,point->y,point->z);
  ROS_INFO_STREAM_THROTTLE(0.1,"active waypoint: ("<<wp->x<<","<<wp->y<<","<<wp->z<<")");
  /* waypoints.push(point2Vector3(*point)); */
}
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  position = point2Vector3(odom->pose.pose.position);
  orientation = odom->pose.pose.orientation;
}

// at SIGINT quit
void quit(int sig) {
  ROS_INFO("User stopped the node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  std::string controllerName;
  ros::init(argc, argv, "benchmark_controller");
  ros::NodeHandle n;
  ros::Duration(5).sleep(); // first ROS_INFO_STREAM did not show on rqt_console and

  string ns = ros::this_node::getNamespace();
  ROS_INFO_STREAM("ns = " << ns);

  signal(SIGINT, quit);

  cmdVelPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Subscriber waypoint_sub = n.subscribe("waypoint", 1000, waypointCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);
  // main loop
  ROS_INFO("Initialized!");

  while(ros::ok()){
    goToWaypoints();
    ros::spinOnce();
  }
  ros::shutdown();
  return 0;
}
