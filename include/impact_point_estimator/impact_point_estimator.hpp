#pragma once

#include "impact_point_estimator/visibility.h"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <eigen3/Eigen/Dense>

namespace impact_point_estimator
{

  class ImpactPointEstimator : public rclcpp::Node
  {
  public:
    IMPACT_POINT_ESTIMATOR_PUBLIC
    explicit ImpactPointEstimator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    IMPACT_POINT_ESTIMATOR_PUBLIC
    explicit ImpactPointEstimator(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void listener_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
    double calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    std::tuple<double, double, double> calculate_centroid();
    void fit_cubic_curve();
    void end_pause();
    void publish_curve_marker(const std::vector<geometry_msgs::msg::Point> &curve_points);
    void publish_points_marker();
    void filter_points(double max_distance);
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double distance_threshold_;
    bool is_predicting_;
    std::vector<geometry_msgs::msg::Point> points_;
  };
} // namespace impact_point_estimator