#include "wheelOdometry.h"

namespace lego_loam
{
WheelOdometry::WheelOdometry()
{
    trackwidth_ = 1.58;
    wheelRLDir = 0;
    wheelRRDir = 0;
    current_time = 0;
    last_time = 0;
    x = 0.0;
    y = 0.0;
    c = -0.037;
}

void WheelOdometry::MsgWheelSpeed(const self_msgs::wheel_speed::ConstPtr &msg)
{
  double radius = 0.0, delta_x = 0.0, delta_y = 0.0, delta_c = 0.0;//, x = 0.0, y = 0.0, c = 0.0;
  double vx = 0.0, vy = 0.0, vth = 0.0;
  double dt = 0.0, midDis = 0.0, leftDis = 0.0, rightDis = 0.0;
  //double c0 = 0.0;
  double rlSpeed = 0.0, rrSpeed = 0.0;

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
  
  std::lock_guard<std::mutex> lock(odom_mutex);
  x += delta_x * cos(c) - delta_y * sin(c);
  y += delta_x * sin(c) + delta_y * cos(c);
  c += delta_c;
 
  last_time = current_time;
}

void WheelOdometry::MsgWheelRLDir(const std_msgs::Int32::ConstPtr &msg)
{
    wheelRLDir = msg->data;
}

void WheelOdometry::MsgWheelRRDir(const std_msgs::Int32::ConstPtr &msg)
{
    wheelRRDir = msg->data;
}

void WheelOdometry::getOdometry(float &fx, float &fy, float &fthita)
{
    std::lock_guard<std::mutex> lock(odom_mutex);
    fx = x;
    fy = y;
    fthita = c;
}

}
