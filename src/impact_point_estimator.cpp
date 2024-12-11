#include "impact_point_estimator/impact_point_estimator.hpp"
#include "impact_point_estimator/filter.hpp"
#include "impact_point_estimator/prediction.hpp"
#include <algorithm>

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
        1000ms,
        std::bind(&ImpactPointEstimator::points_timeout_callback, this));

    last_point_time_ = std::chrono::steady_clock::now();

    motor_pos_ = this->get_parameter("motor_pos").as_double();
    offset_time_ = this->get_parameter("offset_time").as_double();
    curve_points_num_ = this->get_parameter("curve_points_num").as_int();
  }


  void ImpactPointEstimator::listener_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (is_predicting_)
    {
        return;
    }

    geometry_msgs::msg::Point point = msg->pose.position;
    RCLCPP_INFO(this->get_logger(), "point: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);

    // 現在時刻を取得
    auto now = std::chrono::steady_clock::now();

    // ポイントを受信した際にlast_point_time_を更新
    last_point_time_ = now;

    // 初回受信時に基準時刻を設定
    if (!start_time_initialized_)
    {
        t0_ = now;
        start_time_initialized_ = true;
    }

    // 相対時間を計算
    double dt = std::chrono::duration<double>(now - t0_).count();

    // dtをtimestamps_に追加
    timestamps_.emplace_back(dt);

    // 点とdtを検証・追加
    if (!validate_and_add_point(point, dt))
    {
        RCLCPP_WARN(this->get_logger(), "有効な点ではありません");
        return;
    }

    if (points_.size() >= static_cast<size_t>(curve_points_num_))
    {
      RCLCPP_INFO(this->get_logger(), "points_.size(): %zu", points_.size());
      RCLCPP_INFO(this->get_logger(), "curve_points_num_ に達しました。process_points を呼び出します。");
      process_points();
    }
  }

  bool ImpactPointEstimator::validate_and_add_point(const geometry_msgs::msg::Point &point, double dt)
  {
    double limit_z = 0.0;
    if (!filter_.check_point_validity(point, points_, recent_points_, limit_z, distance_threshold_))
    {
      return false;
    }
    points_.emplace_back(point);
    return true;
  }


  void ImpactPointEstimator::process_points()
  {
    // filter_.filter_points(points_, 0.5);

    if (points_.size() < static_cast<size_t>(curve_points_num_))
    {
      RCLCPP_WARN(this->get_logger(), "フィルタリング後の点がcurve_points_num_未満です。");
      return;
    }

    // 時間配列の作成
    std::vector<double> times = generate_time_array();

    // timesの内容を出力
    for (size_t i = 0; i < times.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "times[%zu]: %.6f秒", i, times[i]);
    }

    double x0, y0, z0, vx, vy, vz;
    bool success = prediction_.fit_ballistic_trajectory(points_, times, x0, y0, z0, vx, vy, vz);
    if (!success)
    {
      RCLCPP_WARN(this->get_logger(), "物理モデルによるフィッティングに失敗しました。データをクリアします。");
      clear_data();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "フィッティング結果: x0=%.6f, y0=%.6f, z0=%.6f, vx=%.6f, vy=%.6f, vz=%.6f",
                x0, y0, z0, vx, vy, vz);

    double impact_time, x_impact, y_impact;
    if (!calculate_impact_point(z0, vz, impact_time, x0, y0, vx, vy, x_impact, y_impact))
    {
      RCLCPP_WARN(this->get_logger(), "有効な着弾時間または地点が見つかりませんでした。データをクリアします。");
      clear_data();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "着弾時間: %.2f s, 着弾地点: (%.2f, %.2f)", impact_time, x_impact, y_impact);

    publish_estimated_impact(impact_time, x_impact, y_impact,
                             x0, y0, z0, vx, vy, vz);

    schedule_motor_position(impact_time + offset_time_);

    clear_data();

    pause_processing();
  }

  std::vector<double> ImpactPointEstimator::generate_time_array()
  {
    std::vector<double> times;
    times.reserve(timestamps_.size());

    if (timestamps_.empty())
    {
      return times;
    }

    // 最初の点をt=0として相対時間を計算
    double t0 = timestamps_.front();
    for (const auto &dt : timestamps_)
    {
      double relative_time = dt - t0;
      times.emplace_back(relative_time);
    }

    return times;
  }

  bool ImpactPointEstimator::calculate_impact_point(double z0, double vz, double &impact_time, double x0, double y0, double vx, double vy, double &x_impact, double &y_impact)
  {
    double g = 9.81;
    double a = 0.5 * g;
    double b = -vz;
    double c = z0;

    double discriminant = b * b - 4 * a * c;
    impact_time = -1.0;

    double t1 = 0.0;
    double t2 = 0.0;
    if (discriminant >= 0)
    {
      double sqrt_d = std::sqrt(discriminant);
      t1 = (-b + sqrt_d) / (2 * a);
      t2 = (-b - sqrt_d) / (2 * a);

      if (t1 > 0 && t2 > 0)
        impact_time = std::min(t1, t2);
      else if (t1 > 0)
        impact_time = t1;
      else if (t2 > 0)
        impact_time = t2;
    }

    if (impact_time <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "impact_time が無効です: %f", impact_time);
      RCLCPP_WARN(this->get_logger(), "z0: %f, vz: %f", z0, vz);
      RCLCPP_WARN(this->get_logger(), "x0: %f, y0: %f, vx: %f, vy: %f", x0, y0, vx, vy);
      return false;
    }

    x_impact = x0 + vx * impact_time;
    y_impact = y0 + vy * impact_time;

    return true;
  }

  void ImpactPointEstimator::publish_estimated_impact(
      double impact_time, double x_impact, double y_impact,
      double x0, double y0, double z0,
      double vx, double vy, double vz)
  {
    RCLCPP_INFO(this->get_logger(), "着弾時間: %.2f s, 着弾地点: (%.2f, %.2f)", impact_time, x_impact, y_impact);

    // 可視化用に軌道をプロット
    std::vector<geometry_msgs::msg::Point> trajectory_points = generate_trajectory_points(
        x0, y0, z0, vx, vy, vz, impact_time);
    publish_curve_marker(trajectory_points);

    // 最終着弾点を送信
    geometry_msgs::msg::Point final_point;
    final_point.x = x_impact;
    final_point.y = y_impact;
    final_point.z = 0.0;
    publish_final_pose(final_point);
    publish_points_marker();
  }

  std::vector<geometry_msgs::msg::Point> ImpactPointEstimator::generate_trajectory_points(double x0, double y0, double z0, double vx, double vy, double vz, double impact_time)
  {
    std::vector<geometry_msgs::msg::Point> trajectory_points;
    double t_max = impact_time + 0.5;
    size_t total_points = 100;

    for (size_t i = 0; i < total_points; ++i)
    {
      double t = static_cast<double>(i) / (total_points - 1) * t_max;
      geometry_msgs::msg::Point p;
      p.x = x0 + vx * t;
      p.y = y0 + vy * t;
      p.z = z0 + vz * t - 0.5 * 9.81 * t * t;
      trajectory_points.push_back(p);
    }

    return trajectory_points;
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
    pause_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ImpactPointEstimator::end_pause, this));
  }

  void ImpactPointEstimator::clear_data()
  {
    points_.clear();
    timestamps_.clear();
    RCLCPP_INFO(this->get_logger(), "クリアしました");
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
    if (timer_)
    {
      timer_->cancel();
    }
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