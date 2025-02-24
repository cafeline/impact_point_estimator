#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace impact_point_estimator
{

  class TrajectoryFilter
  {
  public:
    TrajectoryFilter(double v_min, double v_max, const std::vector<double> &direction, double theta_max_deg);

    // 2点間から速度と方向の検証を行う
    bool validateVelocity(const geometry_msgs::msg::Point &prev,
                          const geometry_msgs::msg::Point &curr,
                          double dt);

    // 新規点の有効性（z値、最近の点との関係、最小距離チェック）を判定
    bool isPointValid(const geometry_msgs::msg::Point &point,
                      const std::deque<geometry_msgs::msg::Point> &recentPoints,
                      const geometry_msgs::msg::Point *lastValidPoint,
                      double minDistance,
                      double limit_z);

    // 点群の中から近接点のみを抽出する
    void filterPoints(std::vector<geometry_msgs::msg::Point> &points, double maxDistance) const;

    geometry_msgs::msg::Vector3 getLastVelocity() const { return last_velocity_; }

  private:
    double v_min_;
    double v_max_;
    double theta_max_deg_;
    std::vector<double> normalized_direction_;
    geometry_msgs::msg::Vector3 last_velocity_;

    double calculateDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const;
  };

} // namespace impact_point_estimator
