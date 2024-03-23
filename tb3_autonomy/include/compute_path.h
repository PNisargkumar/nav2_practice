#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ComputePath : public BT::StatefulActionNode
{
public:
  ComputePath(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using ComputePathActionMsg = nav2_msgs::action::ComputePathToPose;
  using PathMsg = nav_msgs::msg::Path;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<ComputePathActionMsg>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<ComputePathActionMsg>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  static BT::PortsList providedPorts();

  // Action Client callback
  void compute_path_callback(const GoalHandleNav::WrappedResult &result);
};