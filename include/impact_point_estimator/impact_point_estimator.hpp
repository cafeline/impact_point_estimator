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
        void publish_curve_marker(const std::vector<geometry_msgs::msg::Point> &curve_points);
        void publish_points_marker();
        void publish_final_pose(const geometry_msgs::msg::Point &final_point);
        void publish_motor_pos(double angle_rad);
        void points_timeout_callback();

        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pos_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr pause_timer_;

        double distance_threshold_;
        bool is_predicting_;
        std::vector<geometry_msgs::msg::Point> points_;

        // 前回の点を保持
        bool has_previous_point_ = false;
        std::deque<geometry_msgs::msg::Point> recent_points_;

        // 追加したメンバー変数
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::steady_clock::time_point end_time_;
        bool start_time_initialized_ = false;
        std::chrono::steady_clock::time_point last_point_time_;
        rclcpp::TimerBase::SharedPtr points_timeout_timer_;

        double motor_pos_;
        double offset_time_;
        int curve_points_num_;

        // フィルタリングと予測処理のオブジェクト
        Filter filter_;
        Prediction prediction_;
    };
} // namespace impact_point_estimator