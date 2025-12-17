#include "indoor_navigation/explore.h"
#include <cmath>

using namespace std::chrono_literals;

ExplorerNode::ExplorerNode()
: Node("explorer_node"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  RCLCPP_INFO(get_logger(), "Explorer node started (TF-based)");

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&ExplorerNode::mapCallback, this, std::placeholders::_1));

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");

  timer_ = create_wall_timer(
    5s, std::bind(&ExplorerNode::explore, this));
}

/* ================= MAP ================= */

void ExplorerNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = msg;
  map_received_ = true;
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
}

/* ================= TF ================= */

bool ExplorerNode::getRobotPose(double & x, double & y)
{
  try {
    auto tf = tf_buffer_.lookupTransform(
      "map", "base_link", tf2::TimePointZero);

    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  }
  catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "TF lookup failed: %s", ex.what());
    return false;
  }
}

/* ================= EXPLORE ================= */

void ExplorerNode::explore()
{
  if (!map_received_ || navigating_) return;

  double robot_x, robot_y;
  if (!getRobotPose(robot_x, robot_y)) return;

  auto & data = map_->data;
  int w = map_->info.width;
  int h = map_->info.height;

  auto frontiers = detectFrontiers(data, w, h);
  if (frontiers.empty()) {
    RCLCPP_INFO(get_logger(), "Exploration complete");
    return;
  }

  auto clusters = clusterFrontiers(frontiers);
  auto best = selectBestFrontier(clusters);
  if (!best.has_value()) return;

  visited_frontiers_.insert(best.value());

  double goal_x = best->second * resolution_ + origin_x_;
  double goal_y = best->first  * resolution_ + origin_y_;

  sendGoal(goal_x, goal_y);
}

/* ================= FRONTIERS ================= */

std::vector<std::pair<int,int>> ExplorerNode::detectFrontiers(
  const std::vector<int8_t> & data, int w, int h)
{
  std::vector<std::pair<int,int>> out;

  for (int r = 1; r < h - 1; r++) {
    for (int c = 1; c < w - 1; c++) {
      int idx = r * w + c;
      if (data[idx] == 0 && hasUnknownNeighbor(r, c, w, h, data)) {
        out.emplace_back(r, c);
      }
    }
  }
  return out;
}

bool ExplorerNode::hasUnknownNeighbor(
  int r, int c, int w, int, const std::vector<int8_t> & data)
{
  for (int dr = -1; dr <= 1; dr++) {
    for (int dc = -1; dc <= 1; dc++) {
      if (dr == 0 && dc == 0) continue;
      if (data[(r+dr)*w + (c+dc)] == -1) return true;
    }
  }
  return false;
}

/* ================= CLUSTER ================= */

std::vector<std::pair<int,int>> ExplorerNode::clusterFrontiers(
  const std::vector<std::pair<int,int>> & frontiers)
{
  std::vector<std::pair<int,int>> clusters;
  constexpr double min_dist = 5.0;

  for (auto & f : frontiers) {
    bool close = false;
    for (auto & c : clusters) {
      if (hypot(f.first-c.first, f.second-c.second) < min_dist) {
        close = true; break;
      }
    }
    if (!close) clusters.push_back(f);
  }
  return clusters;
}

/* ================= SELECT ================= */

std::optional<std::pair<int,int>> ExplorerNode::selectBestFrontier(
  const std::vector<std::pair<int,int>> & frontiers)
{
  double rx, ry;
  if (!getRobotPose(rx, ry)) return std::nullopt;

  int rr = (ry - origin_y_) / resolution_;
  int rc = (rx - origin_x_) / resolution_;

  double best_score = -1e9;
  std::optional<std::pair<int,int>> best;

  for (auto & f : frontiers) {
    if (visited_frontiers_.count(f)) continue;
    double d = hypot(rr-f.first, rc-f.second);
    double score = 1.0 / (d + 1.0);
    if (score > best_score) {
      best_score = score;
      best = f;
    }
  }
  return best;
}

/* ================= NAV ================= */

void ExplorerNode::sendGoal(double x, double y)
{
  if (!nav_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(), "Nav2 unavailable");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.w = 1.0;

  navigating_ = true;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.result_callback =
    std::bind(&ExplorerNode::resultCallback, this, std::placeholders::_1);

  nav_client_->async_send_goal(goal, options);
}

void ExplorerNode::resultCallback(
  const GoalHandleNav::WrappedResult & result)
{
  navigating_ = false;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    RCLCPP_INFO(get_logger(), "Goal reached");
  else
    RCLCPP_WARN(get_logger(), "Navigation failed");
}

/* ================= MAIN ================= */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
