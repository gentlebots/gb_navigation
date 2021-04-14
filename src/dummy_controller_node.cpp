// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>
#include <random>

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DummyNavigationController : public rclcpp::Node
{
public:
  DummyNavigationController()
  : rclcpp::Node("dummy_navigation_controller")
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());

    init_knowledge();

    robot_location_["r2d2"] = "outdoor";
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 " + robot_location_["r2d2"] +")"));

    problem_expert_->setGoal(plansys2::Goal("(and(robot_at r2d2 foodtray))"));

    if (!executor_client_->start_plan_execution()) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"outdoor", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"foodtray", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"near_lemon", "zone"});
  }

  std::string status_to_string(int8_t status) {
    switch (status)
    {
    case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
      return "NOT_EXECUTED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
      return "EXECUTING";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
      return "FAILED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
      return "SUCCEEDED";
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
      return "CANCELLED";
      break;
    default:
      return "UNKNOWN";
      break;
    }
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {
      RCLCPP_INFO(get_logger(), "========================= PLAN FINISHED ==================");
      auto result = executor_client_->getResult();
      if (result.has_value()) {
        RCLCPP_INFO_STREAM(get_logger(), "Plan succesful: " << result.value().success);
        for (const auto & action_info : result.value().action_execution_status) {
                    std::string args;
          rclcpp::Time start_stamp = action_info.start_stamp;
          rclcpp::Time status_stamp = action_info.status_stamp;
          for (const auto & arg : action_info.arguments) {
            args = args + " " + arg;
          } 
          RCLCPP_INFO_STREAM(get_logger(), "Action: " << action_info.action << args << " " << status_to_string(action_info.status) << " " << (status_stamp - start_stamp).seconds() << " secs");
        }
      } else {
        RCLCPP_WARN(get_logger(), "No result for this plan");
      }
    }
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  std::map<std::string, std::string> robot_location_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DummyNavigationController>();

  node->init();

  rclcpp::Rate rate(1);
  while (rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
    node->step();
  }

  rclcpp::shutdown();

  return 0;
}
