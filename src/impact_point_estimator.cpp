#include "impact_point_estimator/impact_point_estimator.hpp"
#include "impact_point_estimator/filter.hpp"
#include "impact_point_estimator/prediction.hpp"

using namespace std::chrono_literals;

namespace impact_point_estimator
{
  ImpactPointEstimator::ImpactPointEstimator(const rclcpp::NodeOptions &options)
      : ImpactPointEstimator("", options) {}

  ImpactPointEstimator::ImpactPointEstimator(const std::string &name_space, const rclcpp::NodeOptions &options)
      : rclcpp::Node("impact_point_estimator", name_space, options),
        is_predicting_(false),
        filter_(),
        prediction_()
  {
    RCLCPP_INFO(this->get_logger(), "impact_point_estimatorの初期化");

    // パラメータの宣言と取得
    this->declare_parameter<double>("distance_threshold", 1.5);
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();

    // サブスクライバーの設定
    subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "tennis_ball", 10, std::bind(&ImpactPointEstimator::listener_callback, this, std::placeholders::_1));

    // パブリッシャーの設定
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_curve", 10);
    points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_points", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/target_pose", 10);
    motor_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("motor/pos", 10);

    points_timeout_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&ImpactPointEstimator::points_timeout_callback, this));

    last_point_time_ = std::chrono::steady_clock::now();

    motor_pos_ = this->get_parameter("motor_pos").as_double();
    offset_time_ = this->get_parameter("offset_time").as_double();
    curve_points_num_ = this->get_parameter("curve_points_num").as_int();
    RCLCPP_INFO(this->get_logger(), "motor_pos: %.2f, offset_time: %.2f, curve_points_num: %d", motor_pos_, offset_time_, curve_points_num_);
  }

  void ImpactPointEstimator::listener_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (is_predicting_)
    {
      return;
    }

    // 最初のポイント受信時にスタートタイムを設定
    if (!start_time_initialized_)
    {
      start_time_ = std::chrono::steady_clock::now();
      start_time_initialized_ = true;
    }

    end_time_ = std::chrono::steady_clock::now();
    last_point_time_ = end_time_;

    geometry_msgs::msg::Point point = msg->pose.position;
    RCLCPP_DEBUG(this->get_logger(), "受信した点: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);

    if (filter_.check_point_validity(point, points_, recent_points_, distance_threshold_))
    {
      points_.emplace_back(point);

      if (points_.size() >= curve_points_num_)
      {
        filter_.filter_points(points_, 0.5);
        if (points_.size() >= curve_points_num_)
        {
          Eigen::VectorXd coeffs_x, coeffs_y, coeffs_z;
          std::vector<geometry_msgs::msg::Point> curve_points = prediction_.fit_cubic_curve(points_, coeffs_x, coeffs_y, coeffs_z);

          // 着弾までの時間を計算
          double target_height = 0.0;
          double time_to_impact = prediction_.calculate_time_to_height(points_, start_time_, end_time_, target_height);
          RCLCPP_INFO(this->get_logger(), "着弾までの時間: %.2f秒", time_to_impact);

          publish_curve_marker(curve_points);
          publish_final_pose(curve_points.back());
          publish_points_marker();

          // 着弾時間分待ってからモーター位置を発行
          if (time_to_impact > 0.0)
          {
            auto delay_ms = static_cast<int>((time_to_impact + offset_time_) * 1000);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(delay_ms),
                [this]()
                {
                  publish_motor_pos(motor_pos_);
                  timer_->cancel();
                });
          }

          points_.clear();

          // 処理を一時停止
          pause_timer_ = this->create_wall_timer(
              1s,
              std::bind(&ImpactPointEstimator::end_pause, this));
        }
      }
    }
  }

  void ImpactPointEstimator::publish_motor_pos(double angle_rad)
  {
    auto message = std_msgs::msg::Float64();
    message.data = angle_rad;
    motor_pos_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published angle: %f rad", angle_rad);
  }

  void ImpactPointEstimator::end_pause()
  {
    is_predicting_ = false;
    timer_->cancel();
    // RCLCPP_INFO(this->get_logger(), "1秒間の処理停止が終了しました。");
  }

  void ImpactPointEstimator::publish_curve_marker(const std::vector<geometry_msgs::msg::Point> &curve_points)
  {
    visualization_msgs::msg::Marker curve_marker;
    curve_marker.header.frame_id = "map";
    curve_marker.header.stamp = this->get_clock()->now();
    curve_marker.ns = "fitted_curve";
    curve_marker.id = 0;
    curve_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    curve_marker.action = visualization_msgs::msg::Marker::ADD;
    curve_marker.scale.x = 0.02; // 線の太さ

    // 色の設定を修正
    curve_marker.color.r = 1.0;
    curve_marker.color.g = 0.0;
    curve_marker.color.b = 0.0;
    curve_marker.color.a = 1.0;

    curve_marker.points = curve_points;
    publisher_->publish(curve_marker);
  }

  void ImpactPointEstimator::publish_points_marker()
  {
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = this->get_clock()->now();
    points_marker.ns = "original_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;

    // スケールの設定を修正
    points_marker.scale.x = 0.07;
    points_marker.scale.y = 0.07;
    points_marker.scale.z = 0.07;

    // 色の設定を修正
    points_marker.color.r = 0.0;
    points_marker.color.g = 1.0;
    points_marker.color.b = 0.5;
    points_marker.color.a = 1.0;

    points_marker.lifetime = rclcpp::Duration(0, 0);

    for (const auto &pt : points_)
    {
      points_marker.points.emplace_back(pt);
    }

    points_publisher_->publish(points_marker);
  }

  void ImpactPointEstimator::publish_final_pose(const geometry_msgs::msg::Point &final_point)
  {
    geometry_msgs::msg::Pose2D target_pose;
    target_pose.x = final_point.x;
    target_pose.y = final_point.y;
    target_pose.theta = 0.0; // 必要に応じて設定
    pose_publisher_->publish(target_pose);
    RCLCPP_INFO(this->get_logger(), "着弾地点: x=%.2f, y=%.2f, theta=%.2f",
                target_pose.x, target_pose.y, target_pose.theta);
  }

  void ImpactPointEstimator::points_timeout_callback()
  {
    auto now = std::chrono::steady_clock::now();
    auto duration_since_last_point = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_point_time_);

    if (duration_since_last_point.count() > 500)
    {
      points_.clear();
    }
  }

} // namespace impact_point_estimator