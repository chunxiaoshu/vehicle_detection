#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>
#include <math.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <time.h>

double joint_position;
bool lds_ready = false;
bool lds_stop = false;
sensor_msgs::PointCloud point_save;
int i = 0;
ros::Time begin_time;

class LdsToPcl {
public:
	LdsToPcl();
	void ldssubCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void ldsPosCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
	void ldsreadysubCallback(const std_msgs::Bool::ConstPtr& lds_ready_msg);
	void ldsstopsubCallback(const std_msgs::Bool::ConstPtr& lds_ready_msg);
private:
	ros::NodeHandle ldsToPclNode;
	ros::Subscriber lds_sub;
	ros::Subscriber lds_ready_sub;
	ros::Subscriber lds_stop_sub;
	ros::Subscriber lds_joint_state;
	ros::Publisher point_cloud_publisher;
};

LdsToPcl::LdsToPcl() {
	lds_sub = ldsToPclNode.subscribe<sensor_msgs::LaserScan> ("/lds_scan_data", 100, &LdsToPcl::ldssubCallback, this);
	lds_ready_sub = ldsToPclNode.subscribe<std_msgs::Bool> ("/lds/ready", 1, &LdsToPcl::ldsreadysubCallback, this);
	lds_stop_sub = ldsToPclNode.subscribe<std_msgs::Bool> ("/lds/stop", 1, &LdsToPcl::ldsstopsubCallback, this);
	lds_joint_state = ldsToPclNode.subscribe<sensor_msgs::JointState> ("/lds_rail/joint_states", 100, &LdsToPcl::ldsPosCallback, this);
	point_cloud_publisher = ldsToPclNode.advertise<sensor_msgs::PointCloud> ("/scan_pointcloud", 100, false);
}

void LdsToPcl::ldssubCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
	const int point_num = 1081;
	const int save_point_num = 400;
	const int save_point_min = (point_num - save_point_num) / 2;
	const int save_point_max = (point_num - save_point_num) / 2;
	const double lds_height = 5.0;
	const double phi = 0.785398;
	double x_, y_, z_, xy_;

	float angle_min = scan->angle_min;
	float angle_max = scan->angle_max;
	float angle_increment = scan->angle_increment;
	float time_increment = scan->time_increment;
	float scan_time = scan->scan_time;
	float range_min = scan->range_min;
	float range_max = scan->range_max;

	if (lds_ready && i == 0) {
		begin_time = ros::Time::now();
	}
	
	if (lds_ready && (!lds_stop) && i < 4000) {
		// std::cout << "angle_min = " << angle_min << std::endl;
		// std::cout << "angle_max = " << angle_max << std::endl;
		// std::cout << "angle_increment = " << angle_increment << std::endl;
		// std::cout << "time_increment = " << time_increment << std::endl;
		// std::cout << "scan_time = " << scan_time << std::endl;
		// std::cout << "range_min = " << range_min << std::endl;
		// std::cout << "range_max = " << range_max << std::endl;

		int point_count_old = point_save.points.size();
		int point_count_new = point_count_old + save_point_num;
		
		point_save.header = scan->header;
		point_save.channels.resize(1);
		point_save.points.resize(point_count_new);
		point_save.channels[0].values.resize(point_count_new);
		

		for (int n = 0; n < save_point_num; ++n) {
			int point_count = n + point_count_old;
			int lds_count = n + save_point_min;
			double lds_angle = angle_min + lds_count * angle_increment;

			if (scan->ranges[lds_count] < range_max && scan->ranges[lds_count] > range_min) {
				z_ = lds_height - scan->ranges[lds_count] * cos(lds_angle);
				xy_ = scan->ranges[lds_count] * sin(lds_angle);
				x_ = joint_position + xy_ * sin(phi);
				y_ = -xy_ * cos(phi);

				point_save.points[point_count].x = x_;
				point_save.points[point_count].y = y_;
				point_save.points[point_count].z = z_;
				// std::cout << "n = " << point_count << "   ldsangel = " << lds_angle
				//  << "   xz = " << xz_ << "   y = " << y_ << "   pos = " << joint_position
				//  << "   x = " << x_ << "   y = " << y_ << std::endl;
			}
			else {
				point_save.points[point_count].x = 0;
				point_save.points[point_count].y = 0;
				point_save.points[point_count].z = 0;
			}
		}

		i++;
		ros::Time end_time = ros::Time::now();
		float during_time = (end_time.sec - begin_time.sec) * 1e3 + (end_time.nsec - begin_time.nsec) / 1e6;
		std::cout << "i = " << i 
			<< ", point_count_new = " << point_count_new
			<< ", joint_position = " << joint_position 
			<< ", scan time = " << during_time 
			<< std::endl;
		point_cloud_publisher.publish(point_save);
	}

	if (i == 4000 || lds_stop) {
		ros::Time end_time = ros::Time::now();
		
		float during_time = (end_time.sec - begin_time.sec) * 1e3 + (end_time.nsec - begin_time.nsec) / 1e6;

		sensor_msgs::PointCloud2 point_save2;
		sensor_msgs::convertPointCloudToPointCloud2(point_save, point_save2);

		pcl::PCLPointCloud2 point_cloud2;
		pcl_conversions::toPCL(point_save2, point_cloud2);

		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		pcl::fromPCLPointCloud2(point_cloud2, point_cloud);

		pcl::io::savePCDFileASCII("./test_pcd.pcd",point_cloud);
		std::cout << "Total time is " << static_cast<float>(during_time) / 1000 << std::endl;
		std::cout << "Saved " << point_cloud.points.size() << " data points to test_pcd.pcd" << std::endl;
		++i;
	}

}

void LdsToPcl::ldsPosCallback(const sensor_msgs::JointState::ConstPtr& joint_states) {
	joint_position = joint_states->position[0];
}

void LdsToPcl::ldsreadysubCallback(const std_msgs::Bool::ConstPtr& lds_ready_msg) {
	lds_ready = lds_ready_msg->data;
}

void LdsToPcl::ldsstopsubCallback(const std_msgs::Bool::ConstPtr& lds_stop_msg) {
	lds_stop = lds_stop_msg->data;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "LdsToPcl");

	LdsToPcl ldstopcl_node;
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();

	return 0;
}
