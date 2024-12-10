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
    // コールバック関数
    void listener_callback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void points_timeout_callback();
    void end_pause();

    // ポイント処理
    bool validate_and_add_point(const geometry_msgs::msg::Point &point, double dt);
    void process_points();
    void clear_data();

    // 時間と軌道計算
    std::vector<double> generate_time_array();
    bool calculate_impact_point(double z0, double vz, double &impact_time,
                                double x0, double y0, double vx, double vy,
                                double &x_impact, double &y_impact);
    std::vector<geometry_msgs::msg::Point> generate_trajectory_points(
        double x0, double y0, double z0,
        double vx, double vy, double vz,
        double impact_time);

    // パブリッシュ関連
    void publish_curve_marker(const std::vector<geometry_msgs::msg::Point> &curve_points);
    void publish_points_marker();
    void publish_final_pose(const geometry_msgs::msg::Point &final_point);
    void publish_motor_pos(double angle_rad);
    void publish_estimated_impact(
        double impact_time, double x_impact, double y_impact,
        double x0, double y0, double z0,
        double vx, double vy, double vz);

    // タイマー制御
    void schedule_motor_position(double delay);
    void pause_processing();

    // ROS 2 通信関連メンバー
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pos_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pause_timer_;

    // 状態管理
    double distance_threshold_;
    bool is_predicting_;
    std::vector<geometry_msgs::msg::Point> points_;
    bool has_previous_point_ = false;
    std::deque<geometry_msgs::msg::Point> recent_points_;

    // 時間管理
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::steady_clock::time_point last_point_time_;
    rclcpp::TimerBase::SharedPtr points_timeout_timer_;
    std::vector<double> timestamps_; // 追加

    std::chrono::steady_clock::time_point t0_; // 追加
    bool start_time_initialized_ = false;      // 追加

    // パラメータ
    double motor_pos_;
    double offset_time_;
    int curve_points_num_;

    // 処理オブジェクト
    Filter filter_;
    Prediction prediction_;
    // std::vector<rclcpp::Time> timestamps_;
  };
} // namespace impact_point_estimator