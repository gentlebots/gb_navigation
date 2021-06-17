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

#include "gb_navigation/behavior_tree_nodes/Search.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace gb_navigation
{

Search::Search(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{

}

void
Search::halt()
{
  std::cout << "Search halt" << std::endl;
}


BT::NodeStatus
Search::tick()
{
  return BT::NodeStatus::RUNNING;
}

}  // namespace gb_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gb_navigation::Search>("Search");
}
