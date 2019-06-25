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
  if ( argc != 4) {
    std::cout << "no enough parameter" << std::endl;
    return -1;
  }
  double lds_position_start = atof(argv[1]);
  double lds_position_stop = atof(argv[2]);
  double speed = atof(argv[3]);
  std::cout << "speed " << speed << std::endl;

  ros::init(argc, argv, "lds_rail");
  ros::NodeHandle lds_rail;
  ros::Publisher lds_position_pub = lds_rail.advertise<std_msgs::Float64>(
          "/lds_rail/lds_rail_position_controller/command", 1);
  ros::Publisher lds_ready_pub = lds_rail.advertise<std_msgs::Bool>(
          "/lds/ready", 1);
  ros::Publisher lds_stop_pub = lds_rail.advertise<std_msgs::Bool>(
          "/lds/stop", 1);

  ros::Subscriber lds_joint_state = lds_rail.subscribe<sensor_msgs::JointState>(
          "/lds_rail/joint_states", 1, ldsPositionCallback);

  int i = 0, t = 0;
  std_msgs::Float64 msg;
  ros::Rate loop_rate(10);

  while (ros::ok() && (joint_position - lds_position_start > 0.01 || joint_position - lds_position_start < -0.01 || i < 3)) {
    msg.data = lds_position_start;
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
    if (joint_position < lds_position_stop) {
      break;
    }
    msg.data = lds_position_start - speed * t;
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

