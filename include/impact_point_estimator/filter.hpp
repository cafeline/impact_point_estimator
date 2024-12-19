#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <deque>
#include <cmath>

class Filter
{
public:
  Filter(double V_min, double V_max, const std::vector<double> &direction_vector, double theta_max_deg);
  bool validate_vel_and_direction(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, double delta_t);
  geometry_msgs::msg::Vector3 get_velocity() const;

  bool check_point_validity(const geometry_msgs::msg::Point &point, std::vector<geometry_msgs::msg::Point> &points, std::deque<geometry_msgs::msg::Point> &recent_points, double limit_z);
  void filter_points(std::vector<geometry_msgs::msg::Point> &points, double max_distance);
  double calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

private:
  double V_min_;
  double V_max_;
  double theta_max_deg_;
  std::vector<double> normalized_direction_;
  geometry_msgs::msg::Vector3 velocity_;
};
