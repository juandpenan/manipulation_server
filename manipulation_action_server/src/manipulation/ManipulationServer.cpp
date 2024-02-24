#include "manipulation/ManipulationServer.hpp"
#include <algorithm>
#include <vector>

namespace manipulation
{
// todos:
// add parameters
// able to prempt the task
// add the rest actions
ManipulationServer::ManipulationServer(const rclcpp::NodeOptions & options)
: rclcpp_cascade_lifecycle::CascadeLifecycleNode("manipulation_action_server", "", options),
  node_{std::make_shared<rclcpp::Node>("manipulation_action_server_executor", options)}
{

  RCLCPP_INFO(this->get_logger(), "Manipulation server created");

}
ManipulationServer::~ManipulationServer()
{
}

CallbackReturn
ManipulationServer::on_configure(const rclcpp_lifecycle::State & state)
{
  // this is for GETTING parameters (todo)
  RCLCPP_INFO(get_logger(), "Configuring manipulation server...");
  action_server_predefined_ = rclcpp_action::create_server<MoveToPredefined>(
    this,
    "move_to_predefined",
    std::bind(
      &ManipulationServer::handle_move_to_predefined_goal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&ManipulationServer::handle_move_to_predefined_cancel, this, std::placeholders::_1),
    std::bind(&ManipulationServer::handle_move_to_predefined_accepted, this, std::placeholders::_1)
  );

  action_server_pick_ = rclcpp_action::create_server<Pick>(
    this,
    "pick",
    std::bind(&ManipulationServer::handle_pick_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ManipulationServer::handle_pick_cancel, this, std::placeholders::_1),
    std::bind(&ManipulationServer::handle_pick_accepted, this, std::placeholders::_1)
  );

  planning_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  interpolation_planner_ =
    std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  // change the name to be a parameter
  // task_.stages()->setName("demo task");
  // task_.loadRobotModel(node_);
  // task_.setProperty("ik_frame", "<gr>ipper_grasping_frame");

  cartesian_planner_ =
    std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner_->setMaxVelocityScalingFactor(1.0);
  cartesian_planner_->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner_->setStepSize(.01);

  sampling_planner_ =
    std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);

  node_thread_ = std::make_unique<std::thread>(
    [this]() {
      executor_.add_node(node_);
      // executor_.spin();
      while(!should_exit_){
        executor_.spin_some();
      }
      executor_.remove_node(node_);
    });

  RCLCPP_INFO(get_logger(), "Manipulation server configured");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleanning up manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn
ManipulationServer::on_shutdown(const rclcpp_lifecycle::State & state)
{
  if (node_thread_->joinable()) {
    node_thread_->join();
    RCLCPP_INFO(get_logger(), "Thread joining...");
  }
  RCLCPP_INFO(get_logger(), "Shutting down manipulation server");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse
ManipulationServer::handle_move_to_predefined_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const manipulation_interfaces::action::MoveToPredefined::Goal> goal)
{

  RCLCPP_INFO(this->get_logger(), "Received goal: move arm to predefined pose %s", goal->goal_pose.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse
ManipulationServer::handle_pick_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const manipulation_interfaces::action::Pick::Goal> goal)
{

  RCLCPP_INFO(this->get_logger(), "Received goal: pick %s", goal->object_goal.id.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::GoalResponse
ManipulationServer::handle_pick_and_place_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const manipulation_interfaces::action::PickAndPlace::Goal> goal)
{

  RCLCPP_INFO(this->get_logger(), "Received goal: pick_and_place %s", goal->object.id.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ManipulationServer::handle_move_to_predefined_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulation_interfaces::action::MoveToPredefined>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal: move arm to predefined pose");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse
ManipulationServer::handle_pick_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulation_interfaces::action::Pick>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal: pick");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::CancelResponse
ManipulationServer::handle_pick_and_place_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<manipulation_interfaces::action::PickAndPlace>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Canceling goal: pick_and_place");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ManipulationServer::handle_move_to_predefined_accepted(
  const std::shared_ptr<GoalHandleMoveToPredefined> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted (move_to_predefined): %s", goal_handle->get_goal()->goal_pose.c_str());

  task_thread_ = std::make_unique<std::thread>(
        std::bind(&ManipulationServer::execute_move_to_predefined, this, std::placeholders::_1),
        goal_handle
  );
  task_thread_->detach();
  
  // auto goal = <goal_handle->get_goal();
  // auto result = std::make_shared<MoveToPredefined::Result>();
  
  // RCLCPP_INFO(get_logger(), "Setting group: %s", goal->group_name.c_str());
  // RCLCPP_INFO(g>et_logger(), "Setting goal: %s", goal->goal_pose.c_str());


  // std::future<moveit::task_constructor::Task> task_future = std::async(
  //   std::launch::async, MoveToPredefinedTask,
  //     goal->group_name,
  //     goal->goal_pose,
  //     node_,
  //     interpolation_planner_);
  
  // task_future.wait();
  // task_ = task_future.get();

  // task_ = MoveToPredefinedTask(
  //   goal->group_name,
  //   goal->goal_pose,
  //   node_,
  //   interpolation_planner_);
  
  // if (SendTask(task_, node_)) {
  //   RCLCPP_INFO(get_logger(), "Goal (move_to_predefined) succeeded");
  //   result->success = true;
  // } else {
  //   RCLCPP_INFO(get_logger(), "Goal (move_to_predefined) failed");
  //   result->success = false;
  // }
  // task_.clear();
  // goal_handle->succeed(result);

}

void
ManipulationServer::execute_move_to_predefined(
  const std::shared_ptr<GoalHandleMoveToPredefined> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal (move_to_predefined): %s", goal_handle->get_goal()->goal_pose.c_str());
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<MoveToPredefined::Result>();
  auto feedback = std::make_shared<MoveToPredefined::Feedback>();

  feedback->msg = "Creating task...";
  goal_handle->publish_feedback(feedback);
  
  task_ = MoveToPredefinedTask(
    goal->group_name,
    goal->goal_pose,
    node_,
    interpolation_planner_);

  feedback->msg = "Executing task...";
  goal_handle->publish_feedback(feedback);

  if (SendTask(task_, node_)) {
    feedback->msg = "Task executed successfully";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Goal (move_to_predefined) succeeded");
    result->success = true;
  } else {
    feedback->msg = "Task failed";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "Goal (move_to_predefined) failed");
    result->success = false;
  }
  task_.clear();
  goal_handle->succeed(result);
}

void
ManipulationServer::handle_pick_accepted(
  const std::shared_ptr<GoalHandlePick> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted (pick): %s", goal_handle->get_goal()->object_goal.id.c_str());

  // attach_object_stage_ptr_ = 
  //   ExecutePick(goal_handle,
  //     node_,
  //     interpolation_planner_,
  //     cartesian_planner_,
  //     planning_interface_,
  //     task_);

  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Pick::Result>();
  moveit::task_constructor::Stage** attach_object_stage;

  task_ = PickTask(
    goal->object_goal,
    attach_object_stage,
    node_,
    interpolation_planner_,
    cartesian_planner_,
    planning_interface_);

  if (SendTask(task_, node_)) {
    // If the gripper is closed, the object has not been picked
    if (IsGripperClosed(node_)) {
      RCLCPP_INFO(get_logger(), "Goal (pick) was executed, but the object is not in the gripper");
      result->success = false;
      has_picked_ = false;
    } else {
      RCLCPP_INFO(get_logger(), "Goal (pick) succeeded");
      result->success = true;
      has_picked_ = true;
    }
  } else {
    RCLCPP_INFO(get_logger(), "Goal (pick) failed");
    result->success = false;
  }
  task_.clear();
  goal_handle->succeed(result);
  
  // if (attach_object_stage_ptr_ != nullptr) {
  //   has_picked_ = true;
  // }
}

void
ManipulationServer::handle_pick_and_place_accepted(
  const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Goal accepted (pick_and_place): %s", goal_handle->get_goal()->object.id.c_str());

  if (!has_picked_) {
    RCLCPP_WARN(this->get_logger(), "No object has been picked yet");
  }

  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PickAndPlace::Result>();

  task_ = PickAndPlaceTask(
    goal->object,
    goal->place_pose,
    node_,
    interpolation_planner_,
    cartesian_planner_,
    sampling_planner_,
    planning_interface_);

  if (SendTask(task_, node_)) {
    RCLCPP_INFO(get_logger(), "Goal (pick_and_place) succeeded");
    result->success = true;
  } else {
    RCLCPP_INFO(get_logger(), "Goal (pick_and_place) failed");
    result->success = false;
  }

  has_picked_ = false;
  task_.clear();
  goal_handle->succeed(result);
}

} // namespace manipulation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(manipulation::ManipulationServer)
