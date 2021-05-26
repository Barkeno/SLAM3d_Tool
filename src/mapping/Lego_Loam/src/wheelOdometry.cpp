#include <stdio.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <vector>
#include <signal.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <self_msgs/wheel_speed.h>

#include <fstream>
using namespace std;

const double wheelbase_ = 2.7;
const double trackwidth_ = 1.58;

const double steer_transmission_ratio_ = 18.8;

//double rlSpeed = 0.0, rrSpeed = 0.0;
double vehicle_vel = 0.0, vehicle_angle = 0.0;
int vehicle_gear = 0;
int wheelRLDir = 0, wheelRRDir = 0;
double current_time, last_time;
double x = 0.0, y = 0.0, c = -0.037;



void MsgWheelSpeed(const self_msgs::wheel_speed::ConstPtr &msg)
{
  double radius = 0.0, delta_x = 0.0, delta_y = 0.0, delta_c = 0.0;//, x = 0.0, y = 0.0, c = 0.0;
  double vx = 0.0, vy = 0.0, vth = 0.0;
  double dt = 0.0, midDis = 0.0, leftDis = 0.0, rightDis = 0.0;
  //double c0 = 0.0;
  double rlSpeed = 0.0, rrSpeed = 0.0;

  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  geometry_msgs::PoseStamped odom_pose;
  geometry_msgs::Vector3Stamped odom_2d;

  rlSpeed = msg->rear_left;
  rrSpeed = msg->rear_right;
  if (last_time < 1e-6) {
      last_time = msg->header.stamp.toSec();
      std::cout << "Odom set last time!" << std::endl;
  }
  current_time = msg->header.stamp.toSec();
  dt = current_time - last_time;

  leftDis = rlSpeed * wheelRLDir * dt;
  rightDis = rrSpeed *wheelRRDir * dt;
  midDis = (leftDis + rightDis) * 0.5;
  delta_c = (rightDis - leftDis) / trackwidth_;
  if (fabs(delta_c) < 1e-6) {
      delta_x = midDis;
      delta_y = 0.0;
  }
  else {
      radius = midDis / delta_c;
      delta_x = radius * sin(delta_c);
      delta_y = radius * (1.0 - cos(delta_c));
  }

  vx = delta_x / dt;
  vy = delta_y / dt;
  vth = delta_c / dt;
  x += delta_x * cos(c) - delta_y * sin(c);
  y += delta_x * sin(c) + delta_y * cos(c);
  c += delta_c;
  // std::cout << "dt = " << dt << std::endl;
  // std::cout << "dx = " << delta_x << std::endl;
  // std::cout << "x = " << x << std::endl;

  odom_quat = tf::createQuaternionMsgFromYaw(c);

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = "odom";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);

  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "map";
  odom.child_frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;
  // ROS_INFO("[odom_pub]: odomX = %lf, odomY = %lf, odomTh = %lf\n", x, y, c);

  // odm_traj.push_back()
  // ofs_pose << fixed << odom.header.stamp << " " << x << " " << y << " " << odom.pose.pose.position.z << " " 
  //                        << odom_quat.w << " " << odom_quat.x << " " << odom_quat.y << " " << odom_quat.z << endl;

  //cout << "current_time: " << msg->header.stamp << ",  odom.header.stamp: " << odom.header.stamp << ", timenow: " << ros::Time::now() << endl;
  
  // odom_2d.header.stamp = ros::Time::now();
  odom_2d.header.stamp = msg->header.stamp;
  odom_2d.vector.x = x*1000;
  odom_2d.vector.y = y*1000;
  odom_2d.vector.z = c;
  // odom_pose.header.stamp = ros::Time::now();
  odom_pose.header.stamp = msg->header.stamp;
  odom_pose.header.frame_id = "odom";
  odom_pose.pose.position.x = x;
  odom_pose.pose.position.y = y;
  odom_pose.pose.position.z = 0.0;
  odom_pose.pose.orientation = odom_quat;
  last_time = current_time;
}

void MsgVehicleVel(const geometry_msgs::Vector3::ConstPtr &msg)
{
    vehicle_vel = msg->x;
    // ROS_INFO("[odom_pub]: vehicle_vel = %lf", vehicle_vel);
}

void MsgVehicleAngle(const std_msgs::Float64::ConstPtr &msg)
{
    vehicle_angle = msg->data;
    // ROS_INFO("[odom_pub]: vehicle_angle = %lf", vehicle_angle);
}

void MsgVehicleGear(const std_msgs::UInt8::ConstPtr &msg)
{
    //P 1 R 2 N 3 D 4
    if (2 == msg->data) {
        vehicle_gear = -1;
    }
    else {
        vehicle_gear = 1;
    }
    // ROS_INFO("[odom_pub]: vehicle_gear = %d", vehicle_gear);
}

void MsgWheelRLDir(const std_msgs::Int32::ConstPtr &msg)
{
    wheelRLDir = msg->data;
}

void MsgWheelRRDir(const std_msgs::Int32::ConstPtr &msg)
{
    wheelRRDir = msg->data;
}


