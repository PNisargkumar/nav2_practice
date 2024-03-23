#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FollowPath : public BT::StatefulActionNode
{
public:
  FollowPath(const std::string &name,
           const BT::NodeConfiguration &config,
           rclcpp::Node::SharedPtr node_ptr);

  using Action = nav2_msgs::action::FollowPath;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<Action>;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<Action>::SharedPtr action_client_ptr_;
  bool done_flag_;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override{};

  static BT::PortsList providedPorts();

  // Action Client callback
  void follow_path_callback(const GoalHandleNav::WrappedResult &result);
};