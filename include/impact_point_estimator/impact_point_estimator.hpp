#pragma once

#include "impact_point_estimator/visibility.h"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <deque>

#include "impact_point_estimator/filter.hpp"
#include "impact_point_estimator/prediction.hpp"

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
    void end_pause();

    void clear_data();

    void publish_estimated_impact(double impact_time, double x_impact, double y_impact, double x0, double y0, double z0, double vx, double vy, double vz);
    void publish_curve_marker(const std::vector<geometry_msgs::msg::Point> &curve_points);
    void publish_points_marker();
    void publish_final_pose(const geometry_msgs::msg::Point &final_point);
    void publish_motor_pos(double angle_rad);
    void schedule_standby_and_reroad(double delay);

    // タイマー制御
    void schedule_motor_position(double delay);
    void pause_processing();
    void process_two_points(const std::vector<geometry_msgs::msg::Point> &points);
    void publish_two_ball_marker(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2);
    void publish_linear_trajectory_marker(const std::vector<geometry_msgs::msg::Point> &trajectory_points);

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pos_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pause_timer_;

    // 状態管理
    bool is_predicting_;
    std::vector<geometry_msgs::msg::Point> points_;
    bool has_previous_point_ = false;
    std::deque<geometry_msgs::msg::Point> recent_points_;

    // 時間管理
    std::chrono::steady_clock::time_point last_point_time_;
    rclcpp::TimerBase::SharedPtr target_pose_timer_;
    rclcpp::TimerBase::SharedPtr standby_timer_;
    // パラメータ
    double motor_pos_;
    double offset_time_;
    int curve_points_num_;
    double standby_pose_x_;
    double standby_pose_y_;
    double reroad_;
    double target_height_;
    double two_points_diff_x_;
    double two_points_diff_y_;

    Filter filter_;
    Prediction prediction_;
  };
} // namespace impact_point_estimator
