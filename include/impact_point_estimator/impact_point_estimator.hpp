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
#include <deque>

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
    void publish_alternate_pose();
    void filter_points(double max_distance);
    void points_timeout_callback();

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double distance_threshold_;
    bool is_predicting_;
    std::vector<geometry_msgs::msg::Point> points_;

    // 前回の点を保持
    geometry_msgs::msg::Point previous_point_;
    bool has_previous_point_ = false;
    std::deque<geometry_msgs::msg::Point> recent_points_;

    // 10秒ごとにPose2Dをパブリッシュするためのタイマー
    rclcpp::TimerBase::SharedPtr alternate_pose_timer_;
    // ポーズのトグル用フラグ
    bool toggle_pose_;

    // 追加したメンバー変数
    std::chrono::steady_clock::time_point last_point_time_;
    rclcpp::TimerBase::SharedPtr points_timeout_timer_;
  };
} // namespace impact_point_estimator