#include "impact_point_estimator/impact_point_estimator.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace impact_point_estimator
{
  ImpactPointEstimator::ImpactPointEstimator(const rclcpp::NodeOptions &options)
      : ImpactPointEstimator("", options) {}

  ImpactPointEstimator::ImpactPointEstimator(const std::string &name_space, const rclcpp::NodeOptions &options)
      : rclcpp::Node("impact_point_estimator", name_space, options),
        is_predicting_(true),
        filter_(),
        prediction_()
  {
    RCLCPP_INFO(this->get_logger(), "impact_point_estimatorの初期化");

    this->declare_parameter<double>("distance_threshold", 1.5);
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    motor_pos_ = this->get_parameter("motor_pos").as_double();
    offset_time_ = this->get_parameter("offset_time").as_double();
    curve_points_num_ = this->get_parameter("curve_points_num").as_int();

    // サブスクライバーの設定
    subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "tennis_ball", 10, std::bind(&ImpactPointEstimator::listener_callback, this, std::placeholders::_1));

    // パブリッシャーの設定
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_curve", 10);
    points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_points", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/target_pose", 10);
    motor_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("motor/pos", 10);

    points_timeout_timer_ = this->create_wall_timer(
        1000ms,
        std::bind(&ImpactPointEstimator::points_timeout_callback, this));

    last_point_time_ = std::chrono::steady_clock::now();
  }

  void ImpactPointEstimator::listener_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (!is_predicting_)
    {
      RCLCPP_INFO(this->get_logger(), "is_predicting_ is false");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "points_size: %d", (int)points_.size());

    geometry_msgs::msg::Point point = msg->pose.position;
    RCLCPP_INFO(this->get_logger(), "point: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);

    // 現在時刻を取得
    auto now = std::chrono::steady_clock::now();

    // ポイントを受信した際にlast_point_time_を更新
    last_point_time_ = now;

    // 初回受信時に基準時刻を設定
    if (!prediction_.is_start_time_initialized())
    {
      prediction_.set_start_time(now);
    }

    // 相対時間を計算
    double dt = prediction_.calculate_relative_time(now);

    // ポイントとdtを検証・追加
    if (!filter_.check_point_validity(point, points_, recent_points_, 0.0, distance_threshold_))
    {
      RCLCPP_WARN(this->get_logger(), "有効な点ではありません");
      return;
    }

    points_.emplace_back(point);
    prediction_.add_timestamp(dt);

    if (points_.size() >= static_cast<size_t>(curve_points_num_))
    {
      RCLCPP_INFO(this->get_logger(), "points_.size(): %zu", points_.size());
      RCLCPP_INFO(this->get_logger(), "curve_points_num_ に達しました。process_points を呼び出します。");
      prediction_.process_points(points_, points_.size(), [this](const PredictionResult &result)
                                 {
        if (result.success)
        {
          publish_estimated_impact(result.impact_time, result.x_impact, result.y_impact,
                                   result.x0, result.y0, result.z0, result.vx, result.vy, result.vz);
          schedule_motor_position(result.impact_time + offset_time_);
        }
        pause_processing(); });
      clear_data();
    }
  }

  void ImpactPointEstimator::points_timeout_callback()
  {
    // 必要に応じて有効化
    // auto now = std::chrono::steady_clock::now();
    // auto duration_since_last_point = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_point_time_);
    // if (duration_since_last_point.count() > 500)
    // {
    //   clear_data();
    // }
  }

  void ImpactPointEstimator::clear_data()
  {
    points_.clear();
    prediction_.clear_timestamps();
    // Prediction内部の開始時刻フラグをリセット
    prediction_.reset_start_time();
    RCLCPP_INFO(this->get_logger(), "データをクリアしました");
  }

  void ImpactPointEstimator::publish_estimated_impact(
      double impact_time, double x_impact, double y_impact,
      double x0, double y0, double z0,
      double vx, double vy, double vz)
  {
    RCLCPP_INFO(this->get_logger(), "着弾時間: %.2f s, 着弾地点: (%.2f, %.2f)", impact_time, x_impact, y_impact);

    // 可視化用に軌道をプロット
    std::vector<geometry_msgs::msg::Point> trajectory_points = prediction_.generate_trajectory_points(x0, y0, z0, vx, vy, vz, impact_time);
    publish_curve_marker(trajectory_points);

    // 最終着弾点を送信
    geometry_msgs::msg::Point final_point;
    final_point.x = x_impact;
    final_point.y = y_impact;
    final_point.z = 0.0;
    publish_final_pose(final_point);
    publish_points_marker();
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

    points_marker.scale.x = 0.07;
    points_marker.scale.y = 0.07;
    points_marker.scale.z = 0.07;

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
    target_pose.theta = 0.0;
    pose_publisher_->publish(target_pose);
    RCLCPP_INFO(this->get_logger(), "着弾地点: x=%.2f, y=%.2f, theta=%.2f",
                target_pose.x, target_pose.y, target_pose.theta);
  }

  void ImpactPointEstimator::schedule_motor_position(double delay)
  {
    if (delay > 0.0)
    {
      auto delay_ms = static_cast<int>(delay * 1000);
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(delay_ms),
          [this]()
          {
            publish_motor_pos(motor_pos_);
            timer_->cancel();
          });
    }
  }

  void ImpactPointEstimator::pause_processing()
  {
    is_predicting_ = false;
    pause_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ImpactPointEstimator::end_pause, this));
  }

  void ImpactPointEstimator::end_pause()
  {
    if (timer_)
    {
      timer_->cancel();
    }
    RCLCPP_INFO(this->get_logger(), "1秒間の処理停止が終了しました。");
    is_predicting_ = true;
  }

  void ImpactPointEstimator::publish_motor_pos(double angle_rad)
  {
    auto message = std_msgs::msg::Float64();
    message.data = angle_rad;
    motor_pos_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published angle: %f rad", angle_rad);
  }

} // namespace impact_point_estimator
