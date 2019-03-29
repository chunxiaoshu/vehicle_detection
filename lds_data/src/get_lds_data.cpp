#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

#define LDS_SAMPLES 760

#define TIME_TICK 0.02

#define LDS_MIN 340
#define LDS_MAX 418


void dataCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("scan_time", msg->scan_time);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_lds_data");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/lds/laser/scan", 1000, dataCallback);

    ros::spin();

    return 0;
}













