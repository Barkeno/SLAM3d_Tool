#ifndef WHEELODOMETRY
#define WHEELODOMETRY

#include <ros/ros.h>
#include <self_msgs/wheel_speed.h>
#include <std_msgs/Int32.h>
#include <mutex>

namespace lego_loam
{

class WheelOdometry{

public:
    double trackwidth_;
    int wheelRLDir;
    int wheelRRDir;
    double current_time;
    double last_time;
    double x, y, c;

    std::mutex odom_mutex;
public:
    WheelOdometry();
    void MsgWheelSpeed(const self_msgs::wheel_speed::ConstPtr &msg);
    void MsgWheelRLDir(const std_msgs::Int32::ConstPtr &msg);
    void MsgWheelRRDir(const std_msgs::Int32::ConstPtr &msg);
    void getOdometry(float &fx, float &fy, float &fthita);

};

}
#endif