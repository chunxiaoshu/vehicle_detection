#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/JointState.h>

double joint_position;

void ldsPositionCallback(const sensor_msgs::JointState::ConstPtr& joint_states) {
	joint_position = joint_states->position[0];
}

int main(int argc, char **argv) {
  if ( argc != 2) {
    std::cout << "no enough parameter" << std::endl;
    return -1;
  }
  double speed = atof(argv[1]);
  std::cout << "speed " << speed << std::endl;

  ros::init(argc, argv, "lds_holder");
  ros::NodeHandle lds_holder;
  ros::Publisher lds_position_pub = lds_holder.advertise<std_msgs::Float64>(
          "/lds_holder/lds_holder_position_controller/command", 1);
  ros::Publisher lds_ready_pub = lds_holder.advertise<std_msgs::Bool>(
          "/lds/ready", 1);
  ros::Publisher lds_stop_pub = lds_holder.advertise<std_msgs::Bool>(
          "/lds/stop", 1);

  ros::Subscriber lds_joint_state = lds_holder.subscribe<sensor_msgs::JointState>(
          "/lds_holder/joint_states", 1, ldsPositionCallback);

  int i = 0, t = 0;
  const double lds_angle_start = 0.45;
  std_msgs::Float64 msg;
  ros::Rate loop_rate(10);

  while (ros::ok() && (joint_position-lds_angle_start > 1e-4 || joint_position-lds_angle_start < -1e-4 || i < 3)) {
    msg.data = lds_angle_start;
    lds_position_pub.publish(msg);
    std::cout << "before, joint position " << joint_position << "    data " << msg.data << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
    ++i;
  }
  
  std_msgs::Bool lds_ready;
  lds_ready.data = true;
  lds_ready_pub.publish(lds_ready);
  std::cout << "at the start" << std::endl;
  
  while (ros::ok()) {
    if (joint_position > 3.14) {
      break;
    }
    // msg.data = 0.017453278 * t;
    msg.data = speed * t + lds_angle_start;    
    lds_position_pub.publish(msg);
    ++t;
    std::cout << "joint position " << joint_position << "    data " << msg.data << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  for (int i = 0; i < 10; ++i) {
    lds_position_pub.publish(msg);
    std::cout << "joint position " << joint_position << "    data " << msg.data << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  std_msgs::Bool lds_stop;
  lds_stop.data = true;
  for (int i = 0; i < 10; ++i) {
    lds_stop_pub.publish(lds_stop);
    loop_rate.sleep();
  }
  std::cout << "stop" << std::endl;

  return 0;
}

