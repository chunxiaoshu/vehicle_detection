#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/JointState.h>

double joint_position;
std_msgs::Float64 require_lds_position;
std_msgs::Bool lds_ready;

void ldsPositionCallback(const sensor_msgs::JointState::ConstPtr& joint_states) {
	joint_position = joint_states->position[0];
}

// void ldsPosRequireCallback(const std_msgs::Float64::ConstPtr&& lds_position) {
//   lds_ready.data = false;
// 	require_lds_position.data = lds_position->data;
// }

int main(int argc, char **argv) {
  if ( argc != 2) {
    std::cout << "no enough parameter" << std::endl;
    return -1;
  }
  double require_pos = atof(argv[1]);
  std::cout << "require lds position " << require_pos << std::endl;

  ros::init(argc, argv, "lds_revolute_multi");
  ros::NodeHandle lds_revolute_multi;

  ros::Subscriber lds_pos = lds_revolute_multi.subscribe<sensor_msgs::JointState>(
          "/lds_revolute/joint_states", 1, ldsPositionCallback);

  // ros::Subscriber lds_require_pos = lds_revolute_multi.subscribe<std_msgs::Float64>(
  //         "/lds_pos", 1, ldsPosRequireCallback);

  lds_ready.data = false;
  ros::Publisher lds_ready_pub = lds_revolute_multi.advertise<std_msgs::Bool>( "/lds/ready", 1 );

  require_lds_position.data = require_pos;
  ros::Publisher lds_position_pub = lds_revolute_multi.advertise<std_msgs::Float64>(
          "/lds_revolute/lds_revolute_position_controller/command", 1);

  int i = 0;
  ros::Rate loop_rate(10);

  while (true) {
    lds_position_pub.publish(require_lds_position);
    std::cout << "require  " << require_pos << ",  at position  " << joint_position << std::endl;

    if ( joint_position - require_pos > -0.01 && joint_position - require_pos < 0.01 && i>13 ) {
      lds_ready.data = true;
      lds_ready_pub.publish(lds_ready);
      break;
    }
    ++i;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}





