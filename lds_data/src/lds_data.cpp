#include "lds_data.h"

int t = 0;

void dataCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  LdsSense sick(msg);
  // std::cout << "angle min is " << msg->angle_min << std::endl;
  // std::cout << "angle max is " << msg->angle_max << std::endl;
  // std::cout << "angle increment is " << msg->angle_increment << std::endl;
  // std::cout << "time increment is " << msg->time_increment << std::endl;
  // std::cout << "scan time is " << msg->scan_time << std::endl;
  // std::cout << "range min is " << msg->range_min << std::endl;
  // std::cout << "range max is " << msg->range_max << std::endl;

  if (t == 3)
  {
    for (int i = LDS_MIN-1; i < LDS_MAX; ++i)
    {
      std::cout << "range at " << i << " is " << sick.point_heigh[i] << std::endl;
    }
  }
  ++t;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_lds_data");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/lds/laser/scan", 1000, dataCallback);

  ros::spin();

  return 0;
}



LdsSense::LdsSense(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  this->min_angle = msg->angle_min;
  this->max_angle = msg->angle_max;
  this->angle_resolution = msg->angle_increment;

  this->min_range = msg->angle_min;
  this->max_range = msg->range_max;
  this->range_resolution = LDS_RANGE_RESOLUTION;

  for (int i = ANGLE_CENTER + 1, j = 1; i <= LDS_MAX; ++i, ++j)
  {
    this->point_heigh[i] = LDS_HEIGHT - msg->ranges[i] * cos(j * this->angle_resolution);
  }
  for (int i = ANGLE_CENTER, j = 0; i >= LDS_MIN; --i, ++j)
  {
    this->point_heigh[i] = LDS_HEIGHT - msg->ranges[i] * cos(j * this->angle_resolution);
  }

  for (int i = ANGLE_CENTER + 1, j = 1; i <= LDS_MAX; ++i, ++j)
  {
    this->point_width[i] = msg->ranges[i] * tan(j * this->angle_resolution);
  }
  for (int i = ANGLE_CENTER, j = 0; i >= LDS_MIN; --i, ++j)
  {
    this->point_width[i] = -msg->ranges[i] * tan(j * this->angle_resolution);
  }

}






/*

Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.




*/









