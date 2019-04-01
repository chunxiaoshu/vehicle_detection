#ifndef _LDS_DATA_H_
#define _LDS_DATA_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <math.h>

#define LDS_SAMPLES 761
#define ANGLE_CENTER 380

#define TIME_TICK 0.02

#define LDS_MIN 340
#define LDS_MAX 420

#define LDS_RANGE_RESOLUTION 0.008
#define LDS_HEIGHT 10


class LdsSense
{
  public: float min_angle;
  public: float max_angle;
  public: float angle_resolution;

  public: float min_range;
  public: float max_range;
  public: float range_resolution;

  public: float point_heigh[LDS_SAMPLES];
  public: float point_width[LDS_SAMPLES];




  public: LdsSense(const sensor_msgs::LaserScan::ConstPtr& msg);


};












#endif