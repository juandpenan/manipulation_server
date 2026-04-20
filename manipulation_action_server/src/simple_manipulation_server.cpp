// Minimal MoveIt-based action server for MoveEndEffector + MoveJoint
#include <thread>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "manipulation_interfaces/action/move_end_effector.hpp"
#include "manipulation_interfaces/action/move_joint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using MoveEndEffector = manipulation_interfaces::action::MoveEndEffector;
using MoveJoint = manipulation_interfaces::action::MoveJoint;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Enable automatic parameter declaration from launch file overrides (needed for MoveIt params)
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("simple_manipulation_server", node_options);

  auto planning_group = node->get_parameter("planning_group").as_string();

  RCLCPP_INFO(
    node->get_logger(), "Simple Manipulation Server starting (group=%s)",
    planning_group.c_str());

  // Create MoveGroupInterface using the node
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node,
    planning_group);
  move_group->setMaxVelocityScalingFactor(1.0);
  move_group->setMaxAccelerationScalingFactor(1.0);

  // Add a collision object in front of the head
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "head_1_link";
  collision_object.id = "head_collision_box";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.03; // Depth
  primitive.dimensions[primitive.BOX_Y] = 0.15; // Width
  primitive.dimensions[primitive.BOX_Z] = 0.08; // Height

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.10; // 5 cm in front
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.04;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "head_1_link";
  attached_object.object = collision_object;

  std::vector<moveit_msgs::msg::AttachedCollisionObject> attached_objects;
  attached_objects.push_back(attached_object);
  planning_scene_interface.applyAttachedCollisionObjects(attached_objects);

  // ─── MoveEndEffector action server ───────────────────────────────────────────
  using GoalHandleMoveEndEffector = rclcpp_action::ServerGoalHandle<MoveEndEffector>;

  auto eef_action_server = rclcpp_action::create_server<MoveEndEffector>(
    node,
    "move_end_effector",
    // goal callback
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveEndEffector::Goal> goal) {
      (void)goal;
      RCLCPP_INFO(
        rclcpp::get_logger("simple_manipulation_server"),
        "Received MoveEndEffector goal");
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

  // ─── MoveJoint action server ─────────────────────────────────────────────────
  // Moves a single named joint to the requested position (radians).
  // group_name in the request is accepted but currently ignored — the server
  // always uses the group it was started with (set via planning_group param).
  using GoalHandleMoveJoint = rclcpp_action::ServerGoalHandle<MoveJoint>;

  auto joint_action_server = rclcpp_action::create_server<MoveJoint>(
    node,
    "move_joint",
    // goal callback
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJoint::Goal> goal) {
      RCLCPP_INFO(
        rclcpp::get_logger("simple_manipulation_server"),
        "Received MoveJoint goal: joint=%s value=%.4f",
        goal->joint_name.c_str(), static_cast<double>(goal->joint_value));
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // cancel callback
    [](const std::shared_ptr<GoalHandleMoveJoint>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    // accepted callback
    [node, move_group](const std::shared_ptr<GoalHandleMoveJoint> goal_handle) {
      std::thread{[node, move_group, goal_handle]() {
          auto goal = goal_handle->get_goal();
          auto feedback = std::make_shared<MoveJoint::Feedback>();
          auto result = std::make_shared<MoveJoint::Result>();

          if (!goal || goal->joint_name.empty()) {
            RCLCPP_ERROR(node->get_logger(), "[MoveJoint] Empty or invalid goal");
            result->success = false;
            goal_handle->abort(result);
            return;
          }

          // Get current joint positions
          feedback->msg = "Reading current joint state";
          goal_handle->publish_feedback(feedback);

          auto current_state = move_group->getCurrentState(10.0);
          if (!current_state) {
            RCLCPP_ERROR(node->get_logger(), "[MoveJoint] Failed to get current robot state");
            result->success = false;
            goal_handle->abort(result);
            return;
          }

          const moveit::core::JointModelGroup * joint_model_group =
          current_state->getJointModelGroup(move_group->getName());

          std::vector<double> joint_positions;
          current_state->copyJointGroupPositions(joint_model_group, joint_positions);

          // Find the joint by name and update its value
          const auto & joint_names = move_group->getJointNames();
          bool found = false;
          for (size_t i = 0; i < joint_names.size(); ++i) {
            if (joint_names[i] == goal->joint_name) {
              joint_positions[i] = static_cast<double>(goal->joint_value);
              found = true;
              break;
            }
          }

          if (!found) {
            RCLCPP_ERROR(
              node->get_logger(),
              "[MoveJoint] Joint '%s' not found in group '%s'",
              goal->joint_name.c_str(), move_group->getName().c_str());
            result->success = false;
            goal_handle->abort(result);
            return;
          }

          // Set joint target and plan
          feedback->msg = "Planning to joint target";
          goal_handle->publish_feedback(feedback);

          bool within_bounds = move_group->setJointValueTarget(joint_positions);
          if (!within_bounds) {
            RCLCPP_WARN(
              node->get_logger(),
              "[MoveJoint] Joint value for '%s' is outside limits — clamping",
              goal->joint_name.c_str());
          }

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          auto plan_res = move_group->plan(plan);
          if (plan_res.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "[MoveJoint] Planning failed (code=%d)", plan_res.val);
            result->success = false;
            goal_handle->abort(result);
            return;
          }

          feedback->msg = "Executing joint plan";
          goal_handle->publish_feedback(feedback);

          auto exec_res = move_group->execute(plan);
          if (exec_res.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(
              node->get_logger(), "[MoveJoint] Execution failed (code=%d)",
              exec_res.val);
            result->success = false;
            goal_handle->abort(result);
            return;
          }

          RCLCPP_INFO(
            node->get_logger(), "[MoveJoint] '%s' moved to %.4f successfully",
            goal->joint_name.c_str(), static_cast<double>(goal->joint_value));
          result->success = true;
          goal_handle->succeed(result);
        }}.detach();
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
