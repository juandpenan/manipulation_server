// Minimal MoveIt-based action server for MoveEndEffector
#include <thread>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "manipulation_interfaces/action/move_end_effector.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

using MoveEndEffector = manipulation_interfaces::action::MoveEndEffector;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Enable automatic parameter declaration from launch file overrides (needed for MoveIt params)
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  auto node = rclcpp::Node::make_shared("simple_manipulation_server", node_options);

  auto planning_group = node->get_parameter("planning_group").as_string();

  RCLCPP_INFO(node->get_logger(), "Simple Manipulation Server starting (group=%s)", planning_group.c_str());

  // Create MoveGroupInterface using the node
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, planning_group);

  using GoalHandleMoveEndEffector = rclcpp_action::ServerGoalHandle<MoveEndEffector>;

  auto action_server = rclcpp_action::create_server<MoveEndEffector>(
    node,
    "move_end_effector",
    // goal callback
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveEndEffector::Goal> goal) {
      (void)goal;
      RCLCPP_INFO(rclcpp::get_logger("simple_manipulation_server"), "Received MoveEndEffector goal");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // cancel callback
    [](const std::shared_ptr<GoalHandleMoveEndEffector>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // accepted callback
    [node, move_group](const std::shared_ptr<GoalHandleMoveEndEffector> goal_handle) {
      std::thread{[node, move_group, goal_handle]() {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveEndEffector::Feedback>();
        auto result = std::make_shared<MoveEndEffector::Result>();

        if (!goal) {
          RCLCPP_ERROR(node->get_logger(), "Empty goal");
          result->success = false;
          goal_handle->abort(result);
          return;
        }

        feedback->msg = "Setting target pose";
        goal_handle->publish_feedback(feedback);

        // set target pose
        move_group->setPoseTarget(goal->pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_res = move_group->plan(plan);
        if (plan_res.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Planning failed (code=%d)", plan_res.val);
          result->success = false;
          goal_handle->abort(result);
          return;
        }

        feedback->msg = "Executing plan";
        goal_handle->publish_feedback(feedback);

        auto exec_res = move_group->execute(plan);
        if (exec_res.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Execution failed (code=%d)", exec_res.val);
          result->success = false;
          goal_handle->abort(result);
          return;
        }

        RCLCPP_INFO(node->get_logger(), "MoveEndEffector succeeded");
        result->success = true;
        goal_handle->succeed(result);
      }}.detach();
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
