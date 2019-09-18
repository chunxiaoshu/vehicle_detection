#include "communication.h"
#include <iostream>
#include <ctime>
#include <cmath>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#define ANGEL_TO_PI(x) (x * 3.14159265359 / 180)

int main(int argc, char* argv[]) {

	
	lds_connect();
	lds_start();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	const double lds_height = 5.0;
	const double angle_min = ANGEL_TO_PI(45);
	const double angle_max = ANGEL_TO_PI(135);
	const double angle_resolution = ANGEL_TO_PI(0.25);

	std::vector<int> range;
	std::vector<int> energy;
	int encoder_pos = 0;
	int data_num = get_lds_data(range, energy, encoder_pos);

	pcl::PointXYZ point;
	if (data_num > 0) {
		double angle_cur = angle_min;
		for (int i = 0; i < data_num; ++i) {
			point.x = static_cast<double>(encoder_pos);
			point.y = static_cast<double>(range[i]) * cos(angle_cur) / 1000;
			point.z = static_cast<double>(range[i]) * sin(angle_cur) / 1000;
			cloud->push_back(point);
			angle_cur += angle_resolution;
		}
			
	}


	lds_stop();
	lds_disconnect();


	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addCoordinateSystem();
	viewer.setBackgroundColor(0.0, 0.0, 0.0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> oricloud_ch(cloud, 255, 255, 255);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, oricloud_ch, "original cloud");
	
	viewer.spin();





	
	return 0;
}