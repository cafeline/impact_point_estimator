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
    motor_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("motor/pos", 10);

    // タイマーの設定
    alternate_pose_timer_ = this->create_wall_timer(
        10s,
        std::bind(&ImpactPointEstimator::publish_alternate_pose, this));

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

    if (check_point_validity(point))
    {
      points_.emplace_back(point);

      if (points_.size() >= curve_points_num_)
      {
        filter_points(0.5);
        if (points_.size() >= curve_points_num_)
        {
          std::vector<geometry_msgs::msg::Point> curve_points = fit_cubic_curve();

          // 着弾までの時間を計算
          double target_height = 0.0;
          double time_to_impact = calculate_time_to_height(target_height);
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
    // 点が2つ未満の場合は計算不可能
    if (points_.size() < 2)
    {
      RCLCPP_WARN(this->get_logger(), "十分な点がありません。着弾時間を計算できません。");
      return -1.0;
    }

    // 計算に使用する最初と最後の点を取得
    auto start_point = points_.front();
    auto end_point = points_.back();

    // 最初の点から最後の点までの経過時間を計算
    std::chrono::duration<double> duration = end_time_ - start_time_;
    double total_time = duration.count(); // 秒単位に変換

    // 初期高さを最初の点のz座標から設定
    double z0 = start_point.z;

    // 初期速度を計算
    // 公式: v0 = (z - z0 + 1/2*g*t^2) / t を使用
    // ここでzは最後の点の高さ、tは経過時間
    double v0 = (end_point.z - z0 + 0.5 * 9.81 * total_time * total_time) / total_time;

    // 計算に使用した初期値をログ出力
    RCLCPP_INFO(this->get_logger(), "初期位置 z0=%.2f m, 初期速度 v0=%.2f m/s, duration=%.2f ms", z0, v0, duration.count() * 1000);

    // 二次方程式の係数を設定
    // z = z0 + v0*t - (1/2)*g*t^2 から
    // (1/2)*g*t^2 - v0*t + (z0 - z_target) = 0 の形に変形
    double a = 0.5 * 9.81;         // 重力加速度の1/2
    double b = -v0;                // 初期速度の負値
    double c = z0 - target_height; // 初期高さと目標高さの差

    // 判別式の計算 (b^2 - 4ac)
    double discriminant = b * b - 4 * a * c;

    // 判別式が負の場合、解なし（目標高さに到達しない）
    if (discriminant < 0)
    {
      RCLCPP_WARN(this->get_logger(), "目標高さに達しません。");
      return -1.0;
    }

    // 二次方程式の解を計算
    double sqrt_discriminant = std::sqrt(discriminant);
    double t1 = (-b + sqrt_discriminant) / (2 * a); // 解1
    double t2 = (-b - sqrt_discriminant) / (2 * a); // 解2

    // 有効な解（着弾時間）を初期化
    double impact_time = -1.0;

    // 正の解のうち、小さい方を選択
    if (t1 >= 0 && t2 >= 0)
    {
      impact_time = std::min(t1, t2);
    }
    else if (t1 >= 0) // t1のみが正の場合
    {
      impact_time = t1;
    }
    else if (t2 >= 0) // t2のみが正の場合
    {
      impact_time = t2;
    }

    // 有効な解が見つかった場合
    if (impact_time >= 0)
    {
      return impact_time;
    }
    else // 有効な解が見つからなかった場合
    {
      RCLCPP_WARN(this->get_logger(), "有効な着弾時間を見つけられませんでした。");
      return -1.0;
    }
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