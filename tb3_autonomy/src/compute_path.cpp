#include "compute_path.h"
#include "yaml-cpp/yaml.h"
#include <string>

ComputePath::ComputePath(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  action_client_ptr_ = rclcpp_action::create_client<ComputePathActionMsg>(node_ptr_, "/compute_path_to_pose");
  done_flag_ = false;
}

BT::PortsList ComputePath::providedPorts()
{
  return {
    BT::InputPort<std::string>("loc"),
    BT::InputPort<std::string>("planner"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "Path created by ComputePathToPose node"),
  };
}

BT::NodeStatus ComputePath::onStart()
{
  // Get location key from port and read YAML file
  BT::Optional<std::string> loc = getInput<std::string>("loc");
  const std::string location_file = node_ptr_->get_parameter("location_file").as_string();

  YAML::Node locations = YAML::LoadFile(location_file);

  std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

  // setup action client
  auto send_goal_options = rclcpp_action::Client<ComputePathActionMsg>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&ComputePath::compute_path_callback, this, std::placeholders::_1);

  // make pose
  auto compute_path_msg = ComputePathActionMsg::Goal();
  compute_path_msg.goal.header.frame_id = "map";
  compute_path_msg.goal.pose.position.x = pose[0];
  compute_path_msg.goal.pose.position.y = pose[1];

  tf2::Quaternion q;
  q.setRPY(0, 0, pose[2]);
  q.normalize();
  compute_path_msg.goal.pose.orientation = tf2::toMsg(q);
  if (auto planner_opt = getInput<std::string>("planner"); planner_opt)
    {
        compute_path_msg.planner_id = *planner_opt;
    }
    else
    {
        compute_path_msg.planner_id = "GridBased";
    }

  // send pose
  done_flag_ = false;
  action_client_ptr_->async_send_goal(compute_path_msg, send_goal_options);
  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2 for calculating path\n");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputePath::onRunning()
{
  if (done_flag_)
  {
  RCLCPP_INFO(node_ptr_->get_logger(), "[%s] Path Calculated\n", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Waiting for calculated path\n");
    return BT::NodeStatus::RUNNING;
  }
}

void ComputePath::compute_path_callback(const GoalHandleNav::WrappedResult &result)
{
  if (result.result)
  {
    auto &path = result.result->path;
    RCLCPP_INFO(node_ptr_->get_logger(), "Sent path with %zu waypoints", path.poses.size());
    /*
    RCLCPP_INFO(node_ptr_->get_logger(), "Printing path contents:");
    for (size_t i = 0; i < path.poses.size(); ++i) {
       const auto& pose = path.poses[i];
       RCLCPP_INFO(node_ptr_->get_logger(), "Waypoint %zu:", i);
       RCLCPP_INFO(node_ptr_->get_logger(), "  Header:");
       RCLCPP_INFO(node_ptr_->get_logger(), "    Stamp: %d.%09d",
                   pose.header.stamp.sec, pose.header.stamp.nanosec);
       RCLCPP_INFO(node_ptr_->get_logger(), "    Frame ID: %s", pose.header.frame_id.c_str());
       RCLCPP_INFO(node_ptr_->get_logger(), "  Pose:");
       RCLCPP_INFO(node_ptr_->get_logger(), "    Position: (%f, %f, %f)",
                   pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
       RCLCPP_INFO(node_ptr_->get_logger(), "    Orientation: (%f, %f, %f, %f)",
                   pose.pose.orientation.x, pose.pose.orientation.y,
                   pose.pose.orientation.z, pose.pose.orientation.w);
       */
    setOutput<nav_msgs::msg::Path>("path", path);
    done_flag_ = true;
  }
}