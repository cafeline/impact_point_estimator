#include "impact_point_estimator/filter.hpp"
#include <rclcpp/rclcpp.hpp>

Filter::Filter(double V_min, double V_max, const std::vector<double> &direction_vector, double theta_max_deg)
    : V_min_(V_min), V_max_(V_max), theta_max_deg_(theta_max_deg)
{
  // 方向ベクトルを正規化
  double norm = 0.0;
  for (const auto &coord : direction_vector)
  {
    norm += coord * coord;
  }
  norm = std::sqrt(norm);
  for (const auto &coord : direction_vector)
  {
    normalized_direction_.push_back(coord / norm);
  }
}

// 2点間から速度ベクトルと速度の大きさを計算
bool Filter::validate_vel_and_direction(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double delta_t)
{
  // 速度ベクトルの計算
  velocity_.x = (p2.x - p1.x) / delta_t;
  velocity_.y = (p2.y - p1.y) / delta_t;
  velocity_.z = (p2.z - p1.z) / delta_t;

  // 速度の大きさ
  double speed = std::sqrt(velocity_.x * velocity_.x + velocity_.y * velocity_.y + velocity_.z * velocity_.z);
  if (speed < V_min_ || speed > V_max_)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Speed %.2f out of range [%.2f, %.2f]", speed, V_min_, V_max_);
    return false;
  }

  // 方向のチェック
  double v_norm = speed;
  if (v_norm == 0.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Velocity norm is zero.");
    return false;
  }

  std::vector<double> v_normalized = {velocity_.x / v_norm, velocity_.y / v_norm, velocity_.z / v_norm};
  double dot_product = 0.0;
  for (size_t i = 0; i < normalized_direction_.size(); ++i)
  {
    dot_product += v_normalized[i] * normalized_direction_[i];
  }

  // 内積を[-1,1]にクランプ
  dot_product = std::max(-1.0, std::min(1.0, dot_product));
  double angle = std::acos(dot_product) * 180.0 / M_PI; // 角度を度に変換

  if (angle > theta_max_deg_)
  {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Direction angle %.2f degrees exceeds max allowed %.2f degrees", angle, theta_max_deg_);
    return false;
  }

  return true;
}

geometry_msgs::msg::Vector3 Filter::get_velocity() const
{
  return velocity_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Filter::check_point_validity(const geometry_msgs::msg::Point &point, std::vector<geometry_msgs::msg::Point> &points, std::deque<geometry_msgs::msg::Point> &recent_points, double limit_z)
{
  if (point.z < limit_z)
  {
    return false;
  }

  // スライディングウィンドウに点を追加
  recent_points.emplace_back(point);
  if (recent_points.size() > 1)
  {
    recent_points.pop_front();
  }

  // x座標が減少しているか確認
  for (size_t i = 1; i < recent_points.size(); ++i)
  {
    if (recent_points[i].x > recent_points[i - 1].x)
    {
      return false;
    }
  }

  // 前の点との距離をチェック
  if (!points.empty())
  {
    double distance = calculate_distance(point, points.back());
    RCLCPP_INFO(rclcpp::get_logger("filter"), "distance: %.2f", distance);
    // if (distance > 1.0 || distance < 0.2)
    if (distance < 0.1)
    {
      return false;
    }
  }

  return true;
}

void Filter::filter_points(std::vector<geometry_msgs::msg::Point> &points, double max_distance)
{
  if (points.empty())
    return;

  std::vector<geometry_msgs::msg::Point> filtered_points;
  for (const auto &pt : points)
  {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &other_pt : points)
    {
      if (&pt == &other_pt)
        continue;
      double dist = calculate_distance(pt, other_pt);
      if (dist < min_dist)
        min_dist = dist;
    }

    if (min_dist <= max_distance)
    {
      filtered_points.push_back(pt);
    }
  }

  points = std::move(filtered_points);
}

double Filter::calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(a.x - b.x, std::hypot(a.y - b.y, a.z - b.z));
}