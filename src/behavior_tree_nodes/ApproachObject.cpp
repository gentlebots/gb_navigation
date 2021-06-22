// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gb_navigation/behavior_tree_nodes/ApproachObject.hpp"

namespace gb_navigation
{

ApproachObject::ApproachObject(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

geometry_msgs::msg::PoseStamped 
ApproachObject::pose2Map(geometry_msgs::msg::PoseStamped input)
{
  tf2::Transform map2frame, frame2obj, map2Object;
  frame2obj.setOrigin({input.pose.position.x,
                      input.pose.position.y,
                      input.pose.position.z});
  frame2obj.setRotation({input.pose.orientation.x,
                        input.pose.orientation.y,
                        input.pose.orientation.z,
                        input.pose.orientation.w});

  geometry_msgs::msg::PoseStamped object_pose;
  try {
    // Check if the transform is available
    auto tf = tf_buffer_->lookupTransform(
       "map", input.header.frame_id, tf2::TimePointZero);
    
    tf2::fromMsg(tf.transform, map2frame);
    map2Object =  map2frame * frame2obj;
    
    object_pose.pose.position.x = map2Object.getOrigin().x();
    object_pose.pose.position.y = map2Object.getOrigin().y();
    object_pose.pose.position.z = map2Object.getOrigin().z();
    object_pose.pose.orientation.x = map2Object.getRotation().x();
    object_pose.pose.orientation.y = map2Object.getRotation().y();
    object_pose.pose.orientation.z = map2Object.getRotation().z();
    object_pose.pose.orientation.w = map2Object.getRotation().w();
    object_pose.header.frame_id = "map";
    object_pose.header.stamp = node_->now();
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(node_->get_logger(), "%s", e.what());
  }

  return object_pose;
}


void
ApproachObject::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput<geometry_msgs::msg::PoseStamped>("object_pose", goal);
  geometry_msgs::msg::PoseStamped pose = pose2Map(goal);
  pose.header.frame_id = "map";
  pose.pose.position.z = 0.0;
  goal_.behavior_tree = ament_index_cpp::get_package_share_directory("gb_navigation") + 
    "/behavior_trees_xml/ApproachObject_nav2_bt.xml";
  goal_.pose = pose;
}

BT::NodeStatus
ApproachObject::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
ApproachObject::on_aborted()
{
  return BT::NodeStatus::SUCCESS;
}


}  // namespace gb_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<gb_navigation::ApproachObject>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<gb_navigation::ApproachObject>(
    "ApproachObject", builder);
}
