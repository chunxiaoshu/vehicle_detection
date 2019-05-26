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
float z_joint=0;
float z_data=0;
class PCL_Call {
     public:
        PCL_Call();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void scanCallback2(const sensor_msgs::JointState::ConstPtr& joint_states);
        void scanCallback3(const std_msgs::Float64::ConstPtr& joint_data);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        ros::Subscriber scan_joint_;
        ros::Subscriber scan_data_;
};



PCL_Call::PCL_Call(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/sick5xx_topic", 100, &PCL_Call::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud> ("/cloud", 100, false);
        scan_joint_ = node_.subscribe<sensor_msgs::JointState> ("/sick5xx_prismatic/joint_states", 100, &PCL_Call::scanCallback2, this);
        scan_data_  = node_.subscribe<std_msgs::Float64> ("/sick5xx_prismatic/sick5xx_prismatic_position_controller/command", 1000, &PCL_Call::scanCallback3, this);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void PCL_Call::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){


    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud cloud_cp;
    static sensor_msgs::PointCloud real_cloud;
    //static sensor_msgs::PointCloud cloud_squad[1000];
    projector_.transformLaserScanToPointCloud("/lds_scan", *scan, cloud, tfListener_);
    // real_cloud=cloud;
    static int i,init_i;
    // if(i%30==1)
    // std::cout << "\ncloud = \n" << real_cloud.points.size() << std::endl;
    int point_count=1138;
    
    int j=0;
    int k=0;
    int freq=2;
    float z_,x_,y_=0;

    const int new_point_count=450;
    const int kan = 180;

    // cloud_cp=cloud;
    // for()
    //real_cloud.points=cloud.points;
    //std::cout << "\ncloud = \n" << real_cloud.points[1] << std::endl;
    // geometry_msgs::Point32 points32[2];

    // points32[1]=real_cloud.points[1];
    // std::cout << "\nz = " << z_data << std::endl;
      
    if((z_data+5)<0.0001&&(z_data+5)>-0.0001&&init_i==0)
    {

        cloud_cp=cloud;
        cloud_cp.points.resize(new_point_count);
        cloud_cp.channels.resize(2);
        cloud_cp.channels[0].values.resize(new_point_count);
        cloud_cp.channels[1].values.resize(new_point_count);
        std::cout << "\ni = \n" << i << std::endl;
        for(j=0;j<new_point_count;j++)
        {
             cloud_cp.points[j]=cloud.points[(point_count-new_point_count-kan)/2+j];
             cloud_cp.channels[0].values[j]=cloud.channels[0].values[(point_count-new_point_count-kan)/2+j];
             cloud_cp.channels[1].values[j]=cloud.channels[1].values[(point_count-new_point_count-kan)/2+j];
        }
        std::cout << "\ni = \n" << i << std::endl;
        cloud=cloud_cp;
    std::cout << "\ni = \n" << i << std::endl;
    if(i==0)
    {

       
    real_cloud.header=cloud.header;
    real_cloud.channels=cloud.channels;
    real_cloud.points=cloud.points;
    for(k=0;k<real_cloud.points.size();k++)
        {
    z_=sqrt((cloud.points[k].x)*(cloud.points[k].x)+(cloud.points[k].y)*(cloud.points[k].y));    
    real_cloud.points[k].x=z_joint;
    real_cloud.points[k].y=(cloud.points[k].y);
    real_cloud.points[k].z=5-(cloud.points[k].x);
            
        }  

    }
    else 
    {
        // cloud_cp=cloud;
        // cloud_cp.points.resize(new_point_count);
        // cloud_cp.channels.resize(2);
        // cloud_cp.channels[0].values.resize(new_point_count);
        // cloud_cp.channels[1].values.resize(new_point_count);
        // for(j=0;j<new_point_count;j++)
        // {
        //      cloud_cp.points[j]=cloud.points[(point_count-new_point_count)/2+j];
        //      cloud_cp.channels[0].values[j]=cloud.channels[0].values[(point_count-new_point_count)/2+j];
        //      cloud_cp.channels[1].values[j]=cloud.channels[1].values[(point_count-new_point_count)/2+j];
        // }
        // std::cout << "\n2222 = \n"  << std::endl;
        // cloud=cloud_cp;
    if(i%freq==0&&init_i==0)
        {
        real_cloud.header=cloud.header;
        
        
        sensor_msgs::PointCloud points32;

        points32.points.resize(new_point_count+real_cloud.points.size());
        points32.channels.resize(2);
        points32.channels[0].values.resize(new_point_count+real_cloud.channels[0].values.size());
        points32.channels[1].values.resize(new_point_count+real_cloud.channels[1].values.size());
        for(k=0;k<real_cloud.points.size();k++)
        {

            points32.points[k]=real_cloud.points[k];
            points32.channels[0].values[k]=real_cloud.channels[0].values[k];
            points32.channels[1].values[k]=real_cloud.channels[1].values[k];

        }   
        for(k=real_cloud.points.size();k<(new_point_count+real_cloud.points.size());k++)
        {   
        x_=cloud.points[k-real_cloud.points.size()].x;
        y_=cloud.points[k-real_cloud.points.size()].y;   
        z_=sqrt(x_*x_+y_*y_);  if(y_<0) z_=-z_;  
    // std::cout << "\nz = " << z_ << std::endl;
    // std::cout << "\nx = " << x_ << std::endl;
    // std::cout << "\ny = " << y_ << std::endl;
        points32.points[k].x=z_joint;
        points32.points[k].y=y_;
        points32.points[k].z=5-x_;

        points32.channels[0].values[k]=real_cloud.channels[0].values[k];
        points32.channels[1].values[k]=real_cloud.channels[1].values[k];
    }      

    real_cloud.points=points32.points;
    real_cloud.channels=points32.channels;
    point_cloud_publisher_.publish(real_cloud);
    // std::cout << "\npublish " << std::endl;
        }
    }
    if(z_joint<=-4.99999&&init_i==0)
    {
        
        pcl::PointCloud<pcl::PointXYZ> real_cloud_pcl;
        pcl::PCLPointCloud2 real_cloud_pcl2;
        sensor_msgs::PointCloud2 real_cloud2;
        sensor_msgs::convertPointCloudToPointCloud2(real_cloud,real_cloud2); 	
        pcl_conversions::toPCL(real_cloud2,real_cloud_pcl2);
        pcl::fromPCLPointCloud2(real_cloud_pcl2,real_cloud_pcl);
        pcl::io::savePCDFileASCII("./test_pcd.pcd",real_cloud_pcl);
        std::cerr<<"Saved "<<real_cloud_pcl.points.size()<<" data points to test_pcd.pcd"<<std::endl;
        init_i=1;
    }

  i++;
  }
    
}

void PCL_Call::scanCallback2(const sensor_msgs::JointState::ConstPtr& joint_states)
{
  sensor_msgs::JointState My_Joint_State;
  My_Joint_State=*joint_states;
  z_joint=My_Joint_State.position[0];
}

void PCL_Call::scanCallback3(const std_msgs::Float64::ConstPtr& joint_states)
{
  std_msgs::Float64 My_Joint_Data;
  My_Joint_Data=*joint_states;
  z_data=My_Joint_Data.data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PCL_Call");
  PCL_Call filter;
  ros::MultiThreadedSpinner s(2);
  ros::spin(s);

  return 0;
}
