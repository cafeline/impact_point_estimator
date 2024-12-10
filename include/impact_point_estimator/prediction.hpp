#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <tuple>
#include <chrono>
#include <eigen3/Eigen/Dense>

namespace impact_point_estimator
{
  class Prediction
  {
  public:
    std::vector<geometry_msgs::msg::Point> fit_cubic_curve(const std::vector<geometry_msgs::msg::Point> &points, Eigen::VectorXd &coeffs_x, Eigen::VectorXd &coeffs_y, Eigen::VectorXd &coeffs_z);
    double calculate_time_to_height(const std::vector<geometry_msgs::msg::Point> &points, std::chrono::steady_clock::time_point start_time, std::chrono::steady_clock::time_point end_time, double target_height);
    std::tuple<double, double, double> calculate_centroid(const std::vector<geometry_msgs::msg::Point> &points);
  };
}