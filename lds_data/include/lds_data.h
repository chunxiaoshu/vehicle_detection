#ifndef _LDS_DATA_H_
#define _LDS_DATA_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <cmath>

#define LDS_SAMPLES 761
#define ANGLE_CENTER 380

#define TIME_TICK 0.02

#define LDS_MIN 340
#define LDS_MAX 420

#define LDS_RANGE_RESOLUTION 0.008
#define LDS_HEIGHT 5.5
#define LDS_OUTOF_RANGE 100


class LdsSensor
{
  public: float min_angle;
  public: float max_angle;
  public: float angle_resolution;

  public: float min_range;
  public: float max_range;
  public: float range_resolution;

  public: float x_offset[LDS_SAMPLES];
  public: float z_offset[LDS_SAMPLES];

  public: LdsSensor(const sensor_msgs::LaserScan::ConstPtr& msg);
  


};



/*
 * For each lds sensor, create coordinate system as follow.

                    ^   z
                    |
                    |
                    |
                    .
                   ...
                  .....
                 .......
                .........
               ...........
              ............. 
             ...............
            ................. --------->  x
 *
 *
 */

/*
 * Messages in sensor_msgs::LaserScan.

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


#endif