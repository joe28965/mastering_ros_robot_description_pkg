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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_CONTROL_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_CONTROL_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class JointControlPrivate;

/// A ackermann drive plugin for car like robots. Subscribes to geometry_msgs/twist

/**
  Example Usage:
  \code{.xml}
    <plugin name="joint_control" filename="libjoint_control.so">

			<ros>
				<namespace>demo</namespace>
			</ros>

			<update_rate>100.0</update_rate>

			<joint_name>name_of_joint</joint_name>
			<topic_name>topic</topic_name>

			<!-- PID tuning -->
			<pid_gain>1500 0 1</pid_gain>
			<i_range>0 0</i_range>

    </plugin>
  \endcode
*/
class JointControl : public gazebo::ModelPlugin
{
public:
  /// Constructor
  JointControl();

  /// Destructor
  ~JointControl();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<JointControlPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_JOINT_CONTROL_HPP_
