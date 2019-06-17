/*
 * Copyright 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/AdvertiseOptions.hh>

#include "PriusPlugin.h"
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>

#include <ros/ros.h>

#define STEERING_RATIO 0.082267516

#define MAXSPEED 37.998337013956565
#define MAXSTEER 0.6458

#define FRONT_TORQUE 859.4004393000001
#define BACK_TORQUE 0

#define FRONT_BRAKE_TORQUE 1031.28052716
#define BACK_BRAKE_TORQUE 687.5203514400001


#define WHEEL_RADIUS 0.31265


namespace gazebo {

  class PriusPluginPrivate {
    public: ros::NodeHandle nh;

    public: ros::Subscriber controlSub;

    /// \brief Pointer to the world
    public: physics::WorldPtr world;

    /// \brief Pointer to the parent model
    public: physics::ModelPtr model;

    /// \brief Transport node
    public: transport::NodePtr gznode;

    /// \brief Ignition transport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport position pub
    public: ignition::transport::Node::Publisher posePub;

    /// \brief Ignition transport console pub
    public: ignition::transport::Node::Publisher consolePub;

    /// \brief Physics update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief Chassis link
    public: physics::LinkPtr chassisLink;

    /// \brief Front left wheel joint
    public: physics::JointPtr flWheelJoint;

    /// \brief Front right wheel joint
    public: physics::JointPtr frWheelJoint;

    /// \brief Rear left wheel joint
    public: physics::JointPtr blWheelJoint;

    /// \brief Rear right wheel joint
    public: physics::JointPtr brWheelJoint;

    /// \brief Front left wheel steering joint
    public: physics::JointPtr flWheelSteeringJoint;

    /// \brief Front right wheel steering joint
    public: physics::JointPtr frWheelSteeringJoint;

    /// \brief PID control for the front left wheel steering joint
    public: common::PID flWheelSteeringPID;

    /// \brief PID control for the front right wheel steering joint
    public: common::PID frWheelSteeringPID;

    /// \brief Last pose msg time
    public: common::Time lastMsgTime;

    /// \brief Last sim time received
    public: common::Time lastSimTime;

    /// \brief speed (m/s) of the car
    public: double SpeedCmd = 0;

    /// \brief Front left wheel desired steering angle (radians)
    public: double flWheelSteeringCmd = 0;

    /// \brief Front right wheel desired steering angle (radians)
    public: double frWheelSteeringCmd = 0;

    /// \brief Steering wheel desired angle (radians)
    public: double handWheelCmd = 0;

    /// \brief Distance distance between front and rear axles
    public: double wheelbaseLength = 0;

    /// \brief Distance distance between front left and right wheels
    public: double frontTrackWidth = 0;

    /// \brief Steering angle of front left wheel at last update (radians)
    public: double flSteeringAngle = 0;

    /// \brief Steering angle of front right wheel at last update (radians)
    public: double frSteeringAngle = 0;

    /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
    public: ignition::math::Vector3d chassisLinearVelocity;

    /// \brief Angular velocity of front left wheel at last update (rad/s)
    public: double flWheelAngularVelocity = 0;

    /// \brief Angular velocity of front right wheel at last update (rad/s)
    public: double frWheelAngularVelocity = 0;

    /// \brief Angular velocity of back left wheel at last update (rad/s)
    public: double blWheelAngularVelocity = 0;

    /// \brief Angular velocity of back right wheel at last update (rad/s)
    public: double brWheelAngularVelocity = 0;

    /// \brief Subscriber to the keyboard topic
    public: transport::SubscriberPtr keyboardSub;

    /// \brief Mutex to protect updates
    public: std::mutex mutex;

    /// \brief Odometer
    public: double odom = 0.0;

    /// \brief Publisher for the world_control topic.
    public: transport::PublisherPtr worldControlPub;
  };
}

using namespace gazebo;

PriusPlugin::PriusPlugin()
    : dataPtr(new PriusPluginPrivate) {
  int argc = 0;
  char *argv = nullptr;
  ros::init(argc, &argv, "PriusPlugin");
  //ros::NodeHandle nh;
  //this->dataPtr->controlSub = nh.subscribe("prius", 10, &PriusPlugin::OnPriusCommand, this);

  this->dataPtr->handWheelCmd = 0;
  this->dataPtr->SpeedCmd = 0;
  
}

PriusPlugin::~PriusPlugin() {
  this->dataPtr->updateConnection.reset();
}

void PriusPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  std::cout << "PriusPlugin loading params" <<std::endl;

  // shortcut to this->dataPtr
  PriusPluginPrivate *dPtr = this->dataPtr.get();

  // load the world and model
  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();
  auto physicsEngine = this->dataPtr->world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  // initial the gazebo node
  this->dataPtr->gznode = transport::NodePtr(new transport::Node());
  this->dataPtr->gznode->Init();

  // get the link and joint name
  std::string chassisLinkName = dPtr->model->GetName() + "::" + _sdf->Get<std::string>("chassis");
  dPtr->chassisLink = dPtr->model->GetLink(chassisLinkName);
  if (!dPtr->chassisLink) {
    std::cout << "could not find chassis link" << std::endl;
    return;
  }

  std::string flWheelJointName = this->dataPtr->model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel");
  this->dataPtr->flWheelJoint = this->dataPtr->model->GetJoint(flWheelJointName);
  if (!this->dataPtr->flWheelJoint) {
    std::cout << "could not find front left wheel joint" <<std::endl;
    return;
  }

  std::string frWheelJointName = this->dataPtr->model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel");
  this->dataPtr->frWheelJoint = this->dataPtr->model->GetJoint(frWheelJointName);
  if (!this->dataPtr->frWheelJoint) {
    std::cout << "could not find front right wheel joint" <<std::endl;
    return;
  }

  std::string blWheelJointName = this->dataPtr->model->GetName() + "::" + _sdf->Get<std::string>("back_left_wheel");
  this->dataPtr->blWheelJoint = this->dataPtr->model->GetJoint(blWheelJointName);
  if (!this->dataPtr->blWheelJoint) {
    std::cout << "could not find back left wheel joint" <<std::endl;
    return;
  }

  std::string brWheelJointName = this->dataPtr->model->GetName() + "::" + _sdf->Get<std::string>("back_right_wheel");
  this->dataPtr->brWheelJoint = this->dataPtr->model->GetJoint(brWheelJointName);
  if (!this->dataPtr->brWheelJoint) {
    std::cout << "could not find back right wheel joint" <<std::endl;
    return;
  }

  std::string flWheelSteeringJointName = this->dataPtr->model->GetName() + "::" + _sdf->Get<std::string>("front_left_wheel_steering");
  this->dataPtr->flWheelSteeringJoint = this->dataPtr->model->GetJoint(flWheelSteeringJointName);
  if (!this->dataPtr->flWheelSteeringJoint) {
    std::cout << "could not find front left steering joint" <<std::endl;
    return;
  } 

  std::string frWheelSteeringJointName = this->dataPtr->model->GetName() + "::" + _sdf->Get<std::string>("front_right_wheel_steering");
  this->dataPtr->frWheelSteeringJoint = this->dataPtr->model->GetJoint(frWheelSteeringJointName);
  if (!this->dataPtr->frWheelSteeringJoint) {
    std::cout << "could not find front right steering joint" <<std::endl;
    return;
  }

  this->dataPtr->SpeedCmd = 1;

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  unsigned int id = 0;
  ignition::math::Vector3d flCenterPos =
    this->dataPtr->flWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();
  ignition::math::Vector3d frCenterPos =
    this->dataPtr->frWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();
  ignition::math::Vector3d blCenterPos =
    this->dataPtr->blWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();
  ignition::math::Vector3d brCenterPos =
    this->dataPtr->brWheelJoint->GetChild()->GetCollision(id)->WorldPose().Pos();

  // track widths are computed first
  ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  this->dataPtr->frontTrackWidth = vec3.Length();
  vec3 = flCenterPos - frCenterPos;

  // to compute wheelbase, first position of axle centers are computed
  ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  this->dataPtr->wheelbaseLength = vec3.Length();

  this->dataPtr->flWheelSteeringPID.SetPGain(1e4);
  this->dataPtr->flWheelSteeringPID.SetIGain(0);
  this->dataPtr->flWheelSteeringPID.SetDGain(3e2);

  this->dataPtr->frWheelSteeringPID.SetPGain(1e4);
  this->dataPtr->frWheelSteeringPID.SetIGain(0);
  this->dataPtr->frWheelSteeringPID.SetDGain(3e2);

  this->dataPtr->flWheelSteeringPID.SetCmdMax(5000);
  this->dataPtr->flWheelSteeringPID.SetCmdMin(-5000);

  this->dataPtr->frWheelSteeringPID.SetCmdMax(5000);
  this->dataPtr->frWheelSteeringPID.SetCmdMin(-5000);

  // start to update
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PriusPlugin::Update, this));

  // subscribe the ros topic
  this->dataPtr->node.Subscribe("/prius/reset", &PriusPlugin::OnReset, this);
  this->dataPtr->node.Subscribe("/prius/stop", &PriusPlugin::OnStop, this);
  this->dataPtr->node.Subscribe("/cmd_vel", &PriusPlugin::OnCmdVel, this);
  this->dataPtr->node.Subscribe("/keypress", &PriusPlugin::OnKeyPressIgn, this);

  // subscribe the gazebo topic
  this->dataPtr->keyboardSub =
    this->dataPtr->gznode->Subscribe("~/keyboard/keypress", &PriusPlugin::OnKeyPress, this, true);

  // publish to ros node
  this->dataPtr->posePub = this->dataPtr->node.Advertise<ignition::msgs::Pose>("/prius/pose");
  this->dataPtr->consolePub = this->dataPtr->node.Advertise<ignition::msgs::Double_V>("/prius/console");

  // publish to gazebo node
  this->dataPtr->worldControlPub = this->dataPtr->gznode->Advertise<msgs::WorldControl>("~/world_control");

  std::cout << "load finish" <<std::endl;
}

void PriusPlugin::OnCmdVel(const ignition::msgs::Pose &_msg) {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->handWheelCmd = _msg.position().x();
  this->dataPtr->SpeedCmd = _msg.position().y();
}

void PriusPlugin::KeyControl(const int _key) {
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // std::cout << "key press"<< _key <<std::endl;

  switch (_key) {
    // w - increase speed
    case 87:
    case 119: {
      this->dataPtr->SpeedCmd += 1.0;
      break;
    }
    // s - decrease speed
    case 83:
    case 115: {
      this->dataPtr->SpeedCmd -= 1.0;
      break;
    }
    // q - stop
    case 113: {
      this->dataPtr->SpeedCmd = 0;
      break;
    }
    // a - steer left
    case 65:
    case 97: {
      this->dataPtr->handWheelCmd += 0.25;
      break;
    }
    // d - steer right
    case 68:
    case 100: {
      this->dataPtr->handWheelCmd -= 0.25;
      break;
    }
    // e - center steering
    case 69:
    case 101: {
      this->dataPtr->handWheelCmd = 0;
      break;
    }
    default: {
      this->dataPtr->handWheelCmd = 0;
      this->dataPtr->SpeedCmd = 0;
      break;
    }
  }
}

void PriusPlugin::OnKeyPress(ConstAnyPtr &_msg) {
  this->KeyControl(_msg->int_value());
}

void PriusPlugin::OnKeyPressIgn(const ignition::msgs::Any &_msg) {
  this->KeyControl(_msg.int_value());
}

void PriusPlugin::OnReset(const ignition::msgs::Any & /*_msg*/) {
  std::cout << "Onreset process" <<std::endl;
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  this->dataPtr->worldControlPub->Publish(msg);
}

void PriusPlugin::OnStop(const ignition::msgs::Any & /*_msg*/) {
  ignition::msgs::StringMsg req;
  ignition::msgs::StringMsg rep;
  bool result = false;
  unsigned int timeout = 5000;
  bool executed = this->dataPtr->node.Request("/priuscup/upload", req, timeout, rep, result);
  if (executed) {
    std::cout << "Result: " << result << std::endl;
    std::cout << rep.data() << std::endl;
  }
  else {
    std::cout << "Service call timed out" << std::endl;
  }
}

void PriusPlugin::Reset() {
  std::cout << "reset all the parameter" <<std::endl;
  this->dataPtr->odom = 0; 
  this->dataPtr->handWheelCmd = 0;;
  this->dataPtr->flSteeringAngle = 0;
  this->dataPtr->frSteeringAngle = 0;
  this->dataPtr->flWheelAngularVelocity = 0;
  this->dataPtr->frWheelAngularVelocity = 0;
  this->dataPtr->blWheelAngularVelocity = 0;
  this->dataPtr->brWheelAngularVelocity = 0;
  this->dataPtr->flWheelSteeringPID.Reset();
  this->dataPtr->frWheelSteeringPID.Reset();
}

void PriusPlugin::Update() {
  // shortcut to this->dataPtr
  PriusPluginPrivate *dPtr = this->dataPtr.get();
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  common::Time curTime = this->dataPtr->world->SimTime();
  double dt = (curTime - this->dataPtr->lastSimTime).Double();
  if (dt < 0) {
    this->Reset();
    return;
  }
  else if (ignition::math::equal(dt, 0.0)) {
    return;
  }

  this->dataPtr->lastSimTime = curTime;

  // get the speed of the body of the car
  dPtr->chassisLinearVelocity = dPtr->chassisLink->WorldCoGLinearVel();
  double linearVel = dPtr->chassisLinearVelocity.Length();    // the speed of car (m/s)
  this->dataPtr->odom += (fabs(linearVel) * dt);          // the distance of car (m)

  // std::cout << "speed is " << linearVel << std::endl;
  // std::cout << "wheel speed is " << dPtr->flWheelAngularVelocity << dPtr->frWheelAngularVelocity << 
  //                                   dPtr->blWheelAngularVelocity << dPtr->brWheelAngularVelocity << std::endl;

  // dPtr->flWheelJoint->SetVelocity(0, 14.0);
  // dPtr->frWheelJoint->SetVelocity(0, 14.0);
  dPtr->blWheelJoint->SetVelocity(0, 3.2 * dPtr->SpeedCmd);
  dPtr->brWheelJoint->SetVelocity(0, 3.2 * dPtr->SpeedCmd);

  // direction control
  // input: this->dataPtr->handWheelCmd, output: the force
  dPtr->flSteeringAngle = dPtr->flWheelSteeringJoint->Position();
  dPtr->frSteeringAngle = dPtr->frWheelSteeringJoint->Position();
  // PID (position) steering
  this->dataPtr->handWheelCmd = ignition::math::clamp(this->dataPtr->handWheelCmd,
        -MAXSTEER / STEERING_RATIO, MAXSTEER / STEERING_RATIO);

  // PID (position) steering joints based on steering position
  // Ackermann steering geometry here
  double tanSteer = tan(this->dataPtr->handWheelCmd * STEERING_RATIO);
  this->dataPtr->flWheelSteeringCmd = atan2(tanSteer,
      1 - this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength * tanSteer);
  this->dataPtr->frWheelSteeringCmd = atan2(tanSteer,
      1 + this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength * tanSteer);

  double flwsError = this->dataPtr->flSteeringAngle - this->dataPtr->flWheelSteeringCmd;
  double flwsCmd = this->dataPtr->flWheelSteeringPID.Update(flwsError, dt);
  this->dataPtr->flWheelSteeringJoint->SetForce(0, flwsCmd);

  double frwsError = this->dataPtr->frSteeringAngle - this->dataPtr->frWheelSteeringCmd;
  double frwsCmd = this->dataPtr->frWheelSteeringPID.Update(frwsError, dt);
  this->dataPtr->frWheelSteeringJoint->SetForce(0, frwsCmd);

  if ((curTime - this->dataPtr->lastMsgTime) > .5) {
    this->dataPtr->posePub.Publish( ignition::msgs::Convert( this->dataPtr->model->WorldPose() ) );

    ignition::msgs::Double_V consoleMsg;

    // linearVel (m/s) = (2*PI*r) * (rad/sec).
    double linearVel = (2.0 * IGN_PI * WHEEL_RADIUS) *
      ((this->dataPtr->flWheelAngularVelocity + this->dataPtr->frWheelAngularVelocity) * 0.5);
    
    // Distance (m)
    this->dataPtr->odom += (fabs(linearVel) * dt);

    // MPH. A speedometer does not go negative.
    consoleMsg.add_data(std::max(linearVel, 0.0));

    // Miles
    consoleMsg.add_data(this->dataPtr->odom);

    this->dataPtr->consolePub.Publish(consoleMsg);

    // Output prius car data.
    this->dataPtr->posePub.Publish(
        ignition::msgs::Convert(this->dataPtr->model->WorldPose()));

    this->dataPtr->lastMsgTime = curTime;
  }
}

GZ_REGISTER_MODEL_PLUGIN(PriusPlugin)
