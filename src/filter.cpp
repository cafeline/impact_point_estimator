#include "impact_point_estimator/filter.hpp"
#include <rclcpp/rclcpp.hpp>

bool Filter::check_point_validity(const geometry_msgs::msg::Point &point, std::vector<geometry_msgs::msg::Point> &points, std::deque<geometry_msgs::msg::Point> &recent_points, double limit_z)
{
  if (point.z < limit_z)
  {
    return false;
  }

  // スライディングウィンドウに点を追加
  recent_points.emplace_back(point);
  if (recent_points.size() > 3)
  {
    recent_points.pop_front();
  }

  // x座標が減少しているか確認
  for (size_t i = 1; i < recent_points.size(); ++i)
  {
    if (recent_points[i].x > recent_points[i - 1].x + 0.05)
    {
      return false;
    }
  }

  // 前の点との距離をチェック
  if (!points.empty())
  {
    double distance = calculate_distance(point, points.back());
    if (distance >= 0.7)
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