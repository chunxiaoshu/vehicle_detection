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
sensor_msgs::PointCloud point_save;

class LdsToPcl {
public:
	LdsToPcl();
	void ldssubCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void ldsPosCallback(const sensor_msgs::JointState::ConstPtr& joint_states);
	void ldsreadysubCallback(const std_msgs::Bool::ConstPtr& lds_ready_msg);
private:
	ros::NodeHandle ldsToPclNode;
	ros::Subscriber lds_sub;
	ros::Subscriber lds_ready_sub;
	ros::Subscriber lds_joint_state;
	ros::Publisher point_cloud_publisher;
  // ros::ServiceClient client;
};

LdsToPcl::LdsToPcl() {
	lds_sub = ldsToPclNode.subscribe<sensor_msgs::LaserScan> ("/lds_scan_data", 100, &LdsToPcl::ldssubCallback, this);
	lds_ready_sub = ldsToPclNode.subscribe<std_msgs::Bool> ("/lds/ready", 1, &LdsToPcl::ldsreadysubCallback, this);
	lds_joint_state = ldsToPclNode.subscribe<sensor_msgs::JointState> ("/lds_revolute/joint_states", 100, &LdsToPcl::ldsPosCallback, this);
	point_cloud_publisher = ldsToPclNode.advertise<sensor_msgs::PointCloud> ("/scan_pointcloud", 100, false);

  // client = ldsToPclNode.serviceClient<beginner_tutorials::AddTwoInts>("lds_revolute_multi");

  // beginner_tutorials::AddTwoInts srv;
  // srv.request.pos = ;
  // client.call(srv);

}

void LdsToPcl::ldssubCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	const int point_num = 1081;
	const double lds_height = 5.0;
	double x_, y_, z_, xy_;
  int i = 0;

	float angle_min = scan->angle_min;
	float angle_max = scan->angle_max;
	float angle_increment = scan->angle_increment;
	float time_increment = scan->time_increment;
	float scan_time = scan->scan_time;
	float range_min = scan->range_min;
	float range_max = scan->range_max;
	
	if ( lds_ready && i == 0) {
		// std::cout << "angle_min = " << angle_min << std::endl;
		// std::cout << "angle_max = " << angle_max << std::endl;
		// std::cout << "angle_increment = " << angle_increment << std::endl;
		// std::cout << "time_increment = " << time_increment << std::endl;
		// std::cout << "scan_time = " << scan_time << std::endl;
		// std::cout << "range_min = " << range_min << std::endl;
		// std::cout << "range_max = " << range_max << std::endl;
		
		point_save.header = scan->header;
		point_save.channels.resize(1);
		point_save.points.resize(point_num);
		point_save.channels[0].values.resize(point_num);
		
		for (int n = 0; n < point_num; ++n) {
			double lds_angle = angle_min + n * angle_increment;

			if (scan->ranges[n] < range_max && scan->ranges[n] > range_min) {
				xy_ = scan->ranges[n] * sin(lds_angle);
				z_ = lds_height - scan->ranges[n] * cos(lds_angle);
				x_ = xy_ * sin(joint_position);
				y_ = xy_ * cos(joint_position); 

				point_save.points[n].x = x_;
				point_save.points[n].y = y_;
				point_save.points[n].z = z_;
				// std::cout << "n = " << point_count << "   ldsangel = " << lds_angle
				//  << "   xz = " << xz_ << "   y = " << y_ << "   pos = " << joint_position
				//  << "   x = " << x_ << "   y = " << y_ << std::endl;
			}
			else {
				point_save.points[n].x = 0;
				point_save.points[n].y = 0;
				point_save.points[n].z = 0;
			}
		}

		std::cout << "measure at the position " << joint_position << std::endl;
		point_cloud_publisher.publish(point_save);

    sensor_msgs::PointCloud2 point_save2;
		sensor_msgs::convertPointCloudToPointCloud2(point_save, point_save2);

		pcl::PCLPointCloud2 point_cloud2;
		pcl_conversions::toPCL(point_save2, point_cloud2);

		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		pcl::fromPCLPointCloud2(point_cloud2, point_cloud);

		pcl::io::savePCDFileASCII("./test_pcd3.pcd",point_cloud);
		std::cout << "Saved " << point_cloud.points.size() << " data points to test_pcd3.pcd" << std::endl;
    i = 1;
	}
}

void LdsToPcl::ldsPosCallback(const sensor_msgs::JointState::ConstPtr& joint_states) {
	joint_position = joint_states->position[0];
}

void LdsToPcl::ldsreadysubCallback(const std_msgs::Bool::ConstPtr& lds_ready_msg) {
	lds_ready = lds_ready_msg->data;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "LdsToPcl");

	LdsToPcl ldstopcl_node;
	ros::MultiThreadedSpinner s(2);
	ros::spin(s);

	return 0;
}
