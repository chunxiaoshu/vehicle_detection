#include "get_lds_data.h"

int t = 0;

void dataCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  LdsSensor sick(msg);
  // std::cout << "angle min is " << msg->angle_min << std::endl;
  // std::cout << "angle max is " << msg->angle_max << std::endl;
  // std::cout << "angle increment is " << msg->angle_increment << std::endl;
  // std::cout << "time increment is " << msg->time_increment << std::endl;
  // std::cout << "scan time is " << msg->scan_time << std::endl;
  // std::cout << "range min is " << msg->range_min << std::endl;
  // std::cout << "range max is " << msg->range_max << std::endl;

  if (t == 3) {
    for (int i = ANGLE_CENTER; i < LDS_SAMPLES; ++i) {
      std::cout << "range at " << i << " is " << sick.z_offset[i] << std::endl;
    }
    std::cout << "angle min is " << msg->angle_min << std::endl;
    std::cout << "angle max is " << msg->angle_max << std::endl;
    std::cout << "angle increment is " << msg->angle_increment << std::endl;
  }
  ++t;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "get_lds_data");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/laser_scan/lds_verticle", 1000, dataCallback);
  ros::spin();

  return 0;
}

LdsSensor::LdsSensor(const sensor_msgs::LaserScan::ConstPtr& msg) {
  this->min_angle = msg->angle_min;
  this->max_angle = msg->angle_max;
  this->angle_resolution = msg->angle_increment;

  this->min_range = msg->angle_min;
  this->max_range = msg->range_max;
  this->range_resolution = LDS_RANGE_RESOLUTION;

  float angel = 0;
  for (int i = ANGLE_CENTER; i < LDS_SAMPLES; ++i) {
    // angel += this->angle_resolution;
    if (msg->ranges[i] <= this->max_range)
    this->z_offset[i] = LDS_HEIGHT - msg->ranges[i] * cos(angel);
    else
    this->z_offset[i] = 0.0;
    angel += this->angle_resolution;
  }

  for (int i = 0, angel = this->min_angle; i < LDS_SAMPLES; ++i) {
    if (msg->ranges[i] <= this->max_range)
    this->x_offset[i] = msg->ranges[i] * tan(angel);
    else
    this->x_offset[i] = LDS_OUTOF_RANGE + 1;
    angel += this->angle_resolution;
  }
}
