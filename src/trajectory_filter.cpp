#include "impact_point_estimator/trajectory_filter.hpp"

namespace impact_point_estimator
{

  TrajectoryFilter::TrajectoryFilter(double v_min, double v_max, const std::vector<double> &direction, double theta_max_deg)
      : v_min_(v_min), v_max_(v_max), theta_max_deg_(theta_max_deg)
  {
    double norm = 0.0;
    for (const auto &d : direction)
    {
      norm += d * d;
    }
    norm = std::sqrt(norm);
    for (const auto &d : direction)
    {
      normalized_direction_.push_back(d / norm);
    }
  }

  bool TrajectoryFilter::validateVelocity(const geometry_msgs::msg::Point &prev,
                                          const geometry_msgs::msg::Point &curr,
                                          double dt)
  {
    last_velocity_.x = (curr.x - prev.x) / dt;
    last_velocity_.y = (curr.y - prev.y) / dt;
    last_velocity_.z = (curr.z - prev.z) / dt;

    double speed = std::sqrt(last_velocity_.x * last_velocity_.x +
                             last_velocity_.y * last_velocity_.y +
                             last_velocity_.z * last_velocity_.z);
    if (speed < v_min_ || speed > v_max_)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryFilter"),
                  "Speed %.2f out of range [%.2f, %.2f]", speed, v_min_, v_max_);
      return false;
    }

    if (speed == 0.0)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryFilter"), "Zero speed detected.");
      return false;
    }
    std::vector<double> vel_norm = {last_velocity_.x / speed, last_velocity_.y / speed, last_velocity_.z / speed};
    double dot = 0.0;
    for (size_t i = 0; i < normalized_direction_.size(); ++i)
    {
      dot += vel_norm[i] * normalized_direction_[i];
    }
    dot = std::max(-1.0, std::min(1.0, dot));
    double angle = std::acos(dot) * 180.0 / M_PI;
    if (angle > theta_max_deg_)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryFilter"),
                  "Direction angle %.2f exceeds max %.2f", angle, theta_max_deg_);
      return false;
    }
    return true;
  }

  bool TrajectoryFilter::isPointValid(const geometry_msgs::msg::Point &point,
                                      const std::deque<geometry_msgs::msg::Point> &recentPoints,
                                      const geometry_msgs::msg::Point *lastValidPoint,
                                      double minDistance,
                                      double limit_z)
  {
    if (point.z < limit_z)
    {
      return false;
    }
    // 最近の点が x 座標で降順になっているかチェック
    if (!recentPoints.empty())
    {
      for (size_t i = 1; i < recentPoints.size(); ++i)
      {
        if (recentPoints[i].x > recentPoints[i - 1].x)
        {
          return false;
        }
      }
    }
    // 最後の有効点との距離チェック（存在する場合）
    if (lastValidPoint)
    {
      double dist = calculateDistance(point, *lastValidPoint);
      if (dist < minDistance)
      {
        return false;
      }
    }
    return true;
  }

  void TrajectoryFilter::filterPoints(std::vector<geometry_msgs::msg::Point> &points, double maxDistance) const
  {
    if (points.empty())
      return;
    std::vector<geometry_msgs::msg::Point> filtered;
    for (const auto &p : points)
    {
      double minDist = std::numeric_limits<double>::max();
      for (const auto &q : points)
      {
        if (&p == &q)
          continue;
        double d = calculateDistance(p, q);
        if (d < minDist)
          minDist = d;
      }
      if (minDist <= maxDistance)
      {
        filtered.push_back(p);
      }
    }
    points = std::move(filtered);
  }

  double TrajectoryFilter::calculateDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const
  {
    return std::hypot(a.x - b.x, std::hypot(a.y - b.y, a.z - b.z));
  }

} // namespace impact_point_estimator
