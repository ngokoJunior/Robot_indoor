#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <vector>
#include <set>

class ExplorerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ExplorerNode();

private:
  /* === Callbacks === */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void explore();
  void sendGoal(double x, double y);
  void resultCallback(const GoalHandleNav::WrappedResult & result);

  /* === Frontier logic === */
  std::vector<std::pair<int,int>> detectFrontiers(
    const std::vector<int8_t> & data, int w, int h);

  bool hasUnknownNeighbor(
    int r, int c, int w, int h, const std::vector<int8_t> & data);

  std::vector<std::pair<int,int>> clusterFrontiers(
    const std::vector<std::pair<int,int>> & frontiers);

  std::optional<std::pair<int,int>> selectBestFrontier(
    const std::vector<std::pair<int,int>> & frontiers);

  bool getRobotPose(double & x, double & y);

  /* === ROS === */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* === TF === */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /* === State === */
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool map_received_{false};
  bool navigating_{false};

  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};

  std::set<std::pair<int,int>> visited_frontiers_;
};
