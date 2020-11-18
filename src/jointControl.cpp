// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <mastering_ros_robot_description_pkg/jointControl.hpp>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sdf/sdf.hh>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class JointControlPrivate
{
public:

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a new is received.
  /// \param[in] _msg Twist command message.
  void OnAngle(std_msgs::msg::Float32::SharedPtr _msg);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command angle
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Pointers to wheel joints.
  gazebo::physics::JointPtr joint_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  // limits of the joint
  double lower_limit_;
	double upper_limit_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// PID control
  gazebo::common::PID pid_;

	/// Goal for joint to get to:
	double target_angle_;
};

JointControl::JointControl()
: impl_(std::make_unique<JointControlPrivate>())
{
}

JointControl::~JointControl()
{
}

void JointControl::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  auto world = impl_->model_->GetWorld();
  auto physicsEngine = world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

	// Get name of the joint
  auto joint_name = _sdf->Get<std::string>("joint_name", "no joint specified").first;
  impl_->joint_ = _model->GetJoint(joint_name);
  if (!impl_->joint_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "joint [%s] not found, plugin will not work.",
      joint_name.c_str());
    impl_->ros_node_.reset();
    return;
  }

	// Get upper and lower limits from model
	impl_->lower_limit_ = impl_->joint_->LowerLimit(0);
	impl_->upper_limit_ = impl_->joint_->UpperLimit(0);

	// Get PID values defined in plugin
  auto pid = _sdf->Get<ignition::math::Vector3d>(
    "pid_gain", ignition::math::Vector3d::Zero).first;
  auto i_range = _sdf->Get<ignition::math::Vector2d>(
    "i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  }	else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

	// Subscribe to the topic
	auto topic_name = _sdf->Get<std::string>("topic_name", "desired_joint_angle").first;
  impl_->angle_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
    topic_name, qos.get_subscription_qos(topic_name, rclcpp::QoS(1)),
    std::bind(&JointControlPrivate::OnAngle, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->angle_sub_->get_topic_name());

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&JointControlPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void JointControl::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
}

void JointControlPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  std::lock_guard<std::mutex> lock(lock_);

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  auto target_angle = target_angle_;

  auto current_angle = joint_->Position(0);

  double angle_diff = current_angle - target_angle;
  double force_cmd = pid_.Update(angle_diff, seconds_since_last_update);

  joint_->SetForce(0, force_cmd);

  last_update_time_ = _info.simTime;
}

void JointControlPrivate::OnAngle(const std_msgs::msg::Float32::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
	double data = _msg->data;
	if(data > upper_limit_) {
		target_angle_ = upper_limit_;
	}	else if(data < lower_limit_) {
		target_angle_ = lower_limit_;
	}	else {
		target_angle_ = data;
	}
}

GZ_REGISTER_MODEL_PLUGIN(JointControl)
}  // namespace gazebo_plugins
