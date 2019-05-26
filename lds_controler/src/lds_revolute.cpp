#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick5xx_revolute");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/sick5xx_revolute/sick5xx_revolute_position_controller/command", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float64 msg;

    if(count<150)
    {
        msg.data = 1;
        chatter_pub.publish(msg);
    }   
    else
    {
        msg.data = -1;
        chatter_pub.publish(msg);
    } 

    ros::spinOnce();
    loop_rate.sleep();
    std::cout << "msg.data = " << msg.data << std::endl;
    ++count;
  }
  return 0;
}

