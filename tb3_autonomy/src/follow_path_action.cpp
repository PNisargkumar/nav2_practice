#include "follow_path_action.h"
#include "yaml-cpp/yaml.h"
#include <string>

FollowPath::FollowPath(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<Action>(node_ptr_, "/follow_path");
  done_flag_ = false;
}

BT::PortsList FollowPath::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path"),
  };
}

BT::NodeStatus FollowPath::onStart()
{
    // Get path from input port
    BT::Optional<nav_msgs::msg::Path> computed_path = getInput<nav_msgs::msg::Path>("path");

    // Check if path is valid
    if (!computed_path)
    {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to receive path from input port");
        done_flag_ = true;
        return BT::NodeStatus::FAILURE;
    }

    // Print received poses
    const auto& path = computed_path.value();
    RCLCPP_INFO(node_ptr_->get_logger(), "Received path with %zu poses:", path.poses.size());

    // Setup action client
    auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&FollowPath::follow_path_callback, this, std::placeholders::_1);

    // Create the action goal
    Action::Goal follow_path_goal;
    follow_path_goal.path = path;
    follow_path_goal.controller_id = "FollowPath";
    follow_path_goal.goal_checker_id = "general_goal_checker";
    follow_path_goal.progress_checker_id = "progress_checker";
    // Send action goal
    done_flag_ = false;
    action_client_ptr_->async_send_goal(follow_path_goal, send_goal_options);

    // Log message indicating that the node is following the received path
    RCLCPP_INFO(node_ptr_->get_logger(), "Following received path");

    return BT::NodeStatus::RUNNING;
}


BT::NodeStatus FollowPath::onRunning()
{
  if (done_flag_)
  {
  RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Goal reached\n", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

void FollowPath::follow_path_callback(const GoalHandleNav::WrappedResult &result)
{
  if (result.result)
  {
    done_flag_ = true;
  }
}