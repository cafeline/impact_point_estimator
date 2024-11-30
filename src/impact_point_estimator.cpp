#include "impact_point_estimator/impact_point_estimator.hpp"

using namespace std::chrono_literals;

namespace impact_point_estimator
{
  ImpactPointEstimator::ImpactPointEstimator(const rclcpp::NodeOptions &options)
      : ImpactPointEstimator("", options) {}

  ImpactPointEstimator::ImpactPointEstimator(const std::string &name_space, const rclcpp::NodeOptions &options)
      : rclcpp::Node("impact_point_estimator", name_space, options),
        toggle_pose_(false),
        is_predicting_(false)
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

    // タイマーの設定
    alternate_pose_timer_ = this->create_wall_timer(
        10s,
        std::bind(&ImpactPointEstimator::publish_alternate_pose, this));

    points_timeout_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&ImpactPointEstimator::points_timeout_callback, this));

    last_point_time_ = std::chrono::steady_clock::now();
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

    if (check_point_validity(point))
    {
      points_.emplace_back(point);

      if (points_.size() >= 3)
      {
        filter_points(0.5);
        if (points_.size() >= 3)
        {
          std::vector<geometry_msgs::msg::Point> curve_points = fit_cubic_curve();

          // 着弾までの時間を計算
          double target_height = 0.0; // 目標高さを設定（例：地面レベル）
          double time_to_impact = calculate_time_to_height(target_height);
          RCLCPP_INFO(this->get_logger(), "着弾までの時間: %.2f秒", time_to_impact);

          publish_curve_marker(curve_points);
          publish_final_pose(curve_points.back());
          publish_points_marker();
          points_.clear();

          // 処理を一時停止
          timer_ = this->create_wall_timer(
              1s,
              std::bind(&ImpactPointEstimator::end_pause, this));
        }
      }
    }
  }

  bool ImpactPointEstimator::check_point_validity(const geometry_msgs::msg::Point &point)
  {
    // スライディングウィンドウに点を追加
    recent_points_.emplace_back(point);
    if (recent_points_.size() > 3)
    {
      recent_points_.pop_front();
    }

    // x座標が減少しているか確認
    for (size_t i = 1; i < recent_points_.size(); ++i)
    {
      if (recent_points_[i].x >= recent_points_[i - 1].x)
      {
        return false;
      }
    }

    // 前の点との距離をチェック
    if (points_.size() >= 1)
    {
      double distance = calculate_distance(point, points_.back());
      if (distance >= 0.5)
      {
        return false;
      }
    }

    return true;
  }
  void ImpactPointEstimator::filter_points(double max_distance)
  {
    if (points_.empty())
      return;

    std::vector<geometry_msgs::msg::Point> filtered_points;
    for (const auto &pt : points_)
    {
      double min_dist = std::numeric_limits<double>::max();
      for (const auto &other_pt : points_)
      {
        if (&pt == &other_pt)
          continue;
        double dist = calculate_distance(pt, other_pt);
        if (dist < min_dist)
          min_dist = dist;
      }

      if (min_dist <= max_distance)
      {
        filtered_points.push_back(pt);
      }
    }

    points_ = std::move(filtered_points);
  }

  double ImpactPointEstimator::calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
  {
    return std::hypot(a.x - b.x, std::hypot(a.y - b.y, a.z - b.z));
  }

  std::tuple<double, double, double> ImpactPointEstimator::calculate_centroid()
  {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto &pt : points_)
    {
      sum_x += pt.x;
      sum_y += pt.y;
      sum_z += pt.z;
    }
    size_t n = points_.size();
    return {sum_x / n, sum_y / n, sum_z / n};
  }

  std::vector<geometry_msgs::msg::Point> ImpactPointEstimator::fit_cubic_curve()
  {
    is_predicting_ = true;

    size_t n = points_.size();
    if (n < 3)
    {
      RCLCPP_WARN(this->get_logger(), "十分な点がありません。カーブのフィッティングをスキップします。");
      is_predicting_ = false;
      return {};
    }

    Eigen::VectorXd t(n);
    for (size_t i = 0; i < n; ++i)
    {
      t(i) = static_cast<double>(i) / (n - 1);
    }

    Eigen::MatrixXd A(n, 4);
    for (size_t i = 0; i < n; ++i)
    {
      A(i, 0) = std::pow(t(i), 3);
      A(i, 1) = std::pow(t(i), 2);
      A(i, 2) = t(i);
      A(i, 3) = 1.0;
    }

    Eigen::VectorXd x(n), y(n), z(n);
    for (size_t i = 0; i < n; ++i)
    {
      x(i) = points_[i].x;
      y(i) = points_[i].y;
      z(i) = points_[i].z;
    }

    coeffs_x_ = A.colPivHouseholderQr().solve(x);
    coeffs_y_ = A.colPivHouseholderQr().solve(y);
    coeffs_z_ = A.colPivHouseholderQr().solve(z);

    // フィットされた曲線の生成
    std::vector<geometry_msgs::msg::Point> curve_points;
    for (size_t i = 0; i < 100; ++i)
    {
      double ti = static_cast<double>(i) / 99.0;
      double xi = coeffs_x_(0) * std::pow(ti, 3) + coeffs_x_(1) * std::pow(ti, 2) + coeffs_x_(2) * ti + coeffs_x_(3);
      double yi = coeffs_y_(0) * std::pow(ti, 3) + coeffs_y_(1) * std::pow(ti, 2) + coeffs_y_(2) * ti + coeffs_y_(3);
      double zi = coeffs_z_(0) * std::pow(ti, 3) + coeffs_z_(1) * std::pow(ti, 2) + coeffs_z_(2) * ti + coeffs_z_(3);

      geometry_msgs::msg::Point pt;
      pt.x = xi;
      pt.y = yi;
      pt.z = zi;
      curve_points.push_back(pt);
    }

    return curve_points;
  }

  double ImpactPointEstimator::calculate_time_to_height(double target_height)
  {
    // カーブのz(t) = a*t^3 + b*t^2 + c*t + d
    for (double t = 0.0; t <= 1.0; t += 0.1)
    {
      double z = coeffs_z_(0) * std::pow(t, 3) + coeffs_z_(1) * std::pow(t, 2) + coeffs_z_(2) * t + coeffs_z_(3);
      RCLCPP_INFO(this->get_logger(), "z(t)=%.2f", z);
      RCLCPP_INFO(this->get_logger(), "target_height=%.2f", target_height);
      if (z <= target_height)
      {
        // tを実際の時間にマッピング
        std::chrono::duration<double> total_duration = end_time_ - start_time_;
        double total_time = total_duration.count(); // 秒単位
        double impact_time = t * total_time;
        return impact_time;
      }
    }
    return -1.0; // 指定した高さに達しない場合
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

  void ImpactPointEstimator::end_pause()
  {
    is_predicting_ = false;
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "1秒間の処理停止が終了しました。");
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

  void ImpactPointEstimator::publish_alternate_pose()
  {
    geometry_msgs::msg::Pose2D pose_msg;

    if (toggle_pose_)
    {
      pose_msg.x = -1.0;
      pose_msg.y = -1.5;
      pose_msg.theta = M_PI / 6.0; // 30°
    }
    else
    {
      pose_msg.x = 1.0;
      pose_msg.y = -1.5;
      pose_msg.theta = -M_PI / 6.0; // -30°
    }

    pose_publisher_->publish(pose_msg);
    toggle_pose_ = !toggle_pose_;
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