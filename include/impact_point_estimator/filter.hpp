#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <deque>
#include <cmath>

class Filter
{
public:
  bool check_point_validity(const geometry_msgs::msg::Point &point, std::vector<geometry_msgs::msg::Point> &points, std::deque<geometry_msgs::msg::Point> &recent_points, double limit_z, double distance_threshold);
  void filter_points(std::vector<geometry_msgs::msg::Point> &points, double max_distance);
  double calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
};