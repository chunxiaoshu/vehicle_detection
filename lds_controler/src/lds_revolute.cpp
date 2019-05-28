#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/JointState.h>

double joint_position;

void ldsPositionCallback(const sensor_msgs::JointState::ConstPtr& joint_states)
{
	joint_position = joint_states->position[0];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick5xx_revolute");
  ros::NodeHandle sick5xx_revolute;

  ros::Publisher lds_position_pub = sick5xx_revolute.advertise<std_msgs::Float64>(
          "/sick5xx_revolute/sick5xx_revolute_position_controller/command", 1);
  ros::Publisher lds_ready_pub = sick5xx_revolute.advertise<std_msgs::Bool>(
          "/sick5xx/ready", 1);

  ros::Subscriber lds_joint_state = sick5xx_revolute.subscribe<sensor_msgs::JointState>(
          "/sick5xx_revolute/joint_states", 1, ldsPositionCallback);

  int i = 0, t = 0;
  const double lds_angle_start = 0.45;
  std_msgs::Float64 msg;
  std_msgs::Bool lds_ready;
  ros::Rate loop_rate(10);
  double speed = atof(argv[1]);
  std::cout << "speed " << speed << std::endl;

  while (ros::ok() && (joint_position-lds_angle_start > 1e-4 || joint_position-lds_angle_start < -1e-4 || i < 3))
  {
    msg.data = lds_angle_start;
    lds_position_pub.publish(msg);
    std::cout << "before, joint position " << joint_position << "    data " << msg.data << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
    ++i;
  }
  
  lds_ready.data = true;
  lds_ready_pub.publish(lds_ready);
  std::cout << "at the start" << std::endl;
  
  while (ros::ok())
  {
    if (joint_position > 3.14)
      break;
    // msg.data = 0.017453278 * t;
    msg.data = speed * t + lds_angle_start;    
    lds_position_pub.publish(msg);
    ++t;
    std::cout << "joint position " << joint_position << "    data " << msg.data << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

