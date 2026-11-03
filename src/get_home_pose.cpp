// Copyright 2026 MIRS
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace simulation
{

// Read the recorder's latched pose without blocking the navigator's executor.
class GetHomePose : public BT::StatefulActionNode
{
public:
  GetHomePose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config),
    node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node"))
  {
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/simulation/home_pose", "Recorded home topic"),
      BT::InputPort<std::string>("global_frame", "map", "Required home frame"),
      BT::InputPort<double>("timeout", 10.0, "Maximum wait in wall seconds"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Recorded home pose")};
  }

  BT::NodeStatus onStart() override
  {
    std::string topic;
    if (!getInput("topic", topic) || topic.empty() ||
      !getInput("global_frame", frame_) || frame_.empty() ||
      !getInput("timeout", timeout_) || !std::isfinite(timeout_) || timeout_ <= 0.0)
    {
      RCLCPP_ERROR(node_->get_logger(), "GetHomePose: invalid topic, frame or timeout");
      return BT::NodeStatus::FAILURE;
    }
    home_.reset();
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
        if (valid(*message)) {
          home_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*message);
        }
      }, options);
    started_ = std::chrono::steady_clock::now();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    executor_.spin_some();
    if (home_) {
      auto pose = *home_;
      // The stored pose is a fixed map location, not a historical TF query.
      pose.header.stamp = node_->now();
      setOutput("pose", pose);
      subscription_.reset();
      return BT::NodeStatus::SUCCESS;
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count() >=
      timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(), "GetHomePose: timed out waiting for a valid home pose");
      subscription_.reset();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    subscription_.reset();
    home_.reset();
  }

private:
  bool valid(const geometry_msgs::msg::PoseStamped & message) const
  {
    const auto & p = message.pose.position;
    const auto & q = message.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return message.header.frame_id == frame_ &&
           std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) && std::abs(norm - 1.0) < 1e-3;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  geometry_msgs::msg::PoseStamped::SharedPtr home_;
  std::string frame_;
  double timeout_{10.0};
  std::chrono::steady_clock::time_point started_;
};

}  // namespace simulation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<simulation::GetHomePose>("GetHomePose");
}
