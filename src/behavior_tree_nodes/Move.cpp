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

#include "gb_navigation/behavior_tree_nodes/Move.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace gb_navigation
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
Move::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput<geometry_msgs::msg::PoseStamped>("goal", goal);
  goal_.behavior_tree = ament_index_cpp::get_package_share_directory("nav2_bt_navigator") + 
    "/behavior_trees/navigate_w_replanning_and_recovery.xml";
  goal_.pose = goal;
}

BT::NodeStatus
Move::on_success()
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
      return std::make_unique<gb_navigation::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<gb_navigation::Move>(
    "Move", builder);
}
