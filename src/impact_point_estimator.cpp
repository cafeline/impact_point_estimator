#include "impact_point_estimator/impact_point_estimator.hpp"

using namespace std::chrono_literals;

namespace impact_point_estimator
{
  ImpactPointEstimator::ImpactPointEstimator(const rclcpp::NodeOptions &options) : ImpactPointEstimator("", options) {}

  ImpactPointEstimator::ImpactPointEstimator(const std::string &name_space, const rclcpp::NodeOptions &options)
      : rclcpp::Node("impact_point_estimator", name_space, options),
        toggle_pose_(false)
  {
    RCLCPP_INFO(this->get_logger(), "impact_point_estimatorの初期化");
    // パラメータの宣言
    this->declare_parameter<double>("distance_threshold", 1.5);
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();

    // サブスクライバー: Markerメッセージを購読
    subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "tennis_ball",
        10,
        std::bind(&ImpactPointEstimator::listener_callback, this, std::placeholders::_1));

    // パブリッシャー: フィッティングされた曲線をMarkerとして公開
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_curve", 10);
    points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_points", 10);

    // 新しいパブリッシャー: 最終点をPose2Dとして公開
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/target_pose", 10);

    // 10秒ごとに交互のPose2Dをパブリッシュするタイマーを設定
    alternate_pose_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&ImpactPointEstimator::publish_alternate_pose, this));
  }

  void ImpactPointEstimator::listener_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (is_predicting_)
    {
      // 予測中のため、メッセージを無視
      return;
    }

    geometry_msgs::msg::Point point = msg->pose.position;
    points_.emplace_back(point);
    // RCLCPP_INFO(this->get_logger(), "点の追加: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);

    // 点間の距離をチェックし、0.3m以上の点を除外
    if (points_.size() >= 5)
    {
      filter_points(0.5);
    }

      if (points_.size() >= 3)
      {
        fit_cubic_curve();
        // RCLCPP_INFO(this->get_logger(), "markerの公開開始");
        publish_points_marker();
      }
    }

  void ImpactPointEstimator::filter_points(double max_distance)
  {
    if (points_.empty())
        return;

    std::vector<geometry_msgs::msg::Point> filtered_points;
    // RCLCPP_INFO(this->get_logger(), "フィルタリング開始: 現在の点数 %zu", points_.size());

    for (size_t i = 0; i < points_.size(); ++i)
    {
        double min_dist = std::numeric_limits<double>::max();
        for (size_t j = 0; j < points_.size(); ++j)
        {
            if (i == j)
                continue;

            double dist = calculate_distance(points_[i], points_[j]);
            if (dist < min_dist)
                min_dist = dist;
        }

        if (min_dist <= max_distance)
        {
            filtered_points.push_back(points_[i]);
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "点を除去: x=%.2f, y=%.2f, z=%.2f", points_[i].x, points_[i].y, points_[i].z);
        }
    }

    points_ = std::move(filtered_points);
    // RCLCPP_INFO(this->get_logger(), "フィルタリング後の点数: %zu", points_.size());
  }

  double ImpactPointEstimator::calculate_distance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
  {
    return std::sqrt(std::pow(a.x - b.x, 2) +
                     std::pow(a.y - b.y, 2) +
                     std::pow(a.z - b.z, 2));
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

  void ImpactPointEstimator::fit_cubic_curve()
  {
    is_predicting_ = true;

    size_t n = points_.size();
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

    Eigen::VectorXd coeffs_x = A.colPivHouseholderQr().solve(x);
    Eigen::VectorXd coeffs_y = A.colPivHouseholderQr().solve(y);
    Eigen::VectorXd coeffs_z = A.colPivHouseholderQr().solve(z);

    // フィットされた曲線を生成
    std::vector<geometry_msgs::msg::Point> curve_points;
    for (size_t i = 0; i < 100; ++i)
    {
      double ti = static_cast<double>(i) / 99.0;
      double xi = coeffs_x(0) * std::pow(ti, 3) + coeffs_x(1) * std::pow(ti, 2) + coeffs_x(2) * ti + coeffs_x(3);
      double yi = coeffs_y(0) * std::pow(ti, 3) + coeffs_y(1) * std::pow(ti, 2) + coeffs_y(2) * ti + coeffs_y(3);
      double zi = coeffs_z(0) * std::pow(ti, 3) + coeffs_z(1) * std::pow(ti, 2) + coeffs_z(2) * ti + coeffs_z(3);

      geometry_msgs::msg::Point pt;
      pt.x = xi;
      pt.y = yi;
      pt.z = zi;
      curve_points.push_back(pt);
    }

    publish_curve_marker(curve_points);
    RCLCPP_INFO(this->get_logger(), "フィットされた曲線のMarkerを公開しました。");

    // 最終ポイントをPose2Dで公開
    geometry_msgs::msg::Pose2D target_pose;
    target_pose.x = curve_points.back().x;
    target_pose.y = curve_points.back().y;
    target_pose.theta = 0.0; // 必要に応じて角度を設定
    // pose_publisher_->publish(target_pose);
    // RCLCPP_INFO(this->get_logger(), "最終点Pose2Dとして公開: x=%.2f, y=%.2f, theta=%.2f",
    //             target_pose.x, target_pose.y, target_pose.theta);

    // 点リストをリセット
    points_.clear();

    // 1秒間処理を停止するためのタイマーを開始
    timer_ = this->create_wall_timer(
        1s,
        std::bind(&ImpactPointEstimator::end_pause, this));
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
    curve_marker.color.r = 1.0;
    curve_marker.color.g = 0.0;
    curve_marker.color.b = 0.0;
    curve_marker.color.a = 1.0;

    curve_marker.points = curve_points;

    publisher_->publish(curve_marker);
  }

  void ImpactPointEstimator::publish_points_marker()
  {
    RCLCPP_INFO(this->get_logger(), "markerの公開");

    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = "map";
    points_marker.header.stamp = this->get_clock()->now();
    points_marker.ns = "original_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.scale.x = 0.05; // 各ポイントのサイズ
    points_marker.scale.y = 0.05;
    points_marker.scale.z = 0.05;
    points_marker.color.r = 0.0;
    points_marker.color.g = 1.0;
    points_marker.color.b = 0.0;
    points_marker.color.a = 1.0;

    RCLCPP_INFO(this->get_logger(), "marker size: %zu", points_.size());
    for (const auto &pt : points_)
    {
      geometry_msgs::msg::Point point;
      point.x = pt.x;
      point.y = pt.y;
      point.z = pt.z;
      RCLCPP_INFO(this->get_logger(), "marker点の追加: x=%.2f, y=%.2f, z=%.2f", point.x, point.y, point.z);
      points_marker.points.push_back(point);
    }

    points_publisher_->publish(points_marker);
  }

  void ImpactPointEstimator::publish_alternate_pose()
  {
    geometry_msgs::msg::Pose2D pose_msg;

    if (toggle_pose_)
    {
      // (-1, -1.5, 30°)
      pose_msg.x = -1.0;
      pose_msg.y = -1.5;
      pose_msg.theta = 30.0 * M_PI / 180.0; // ラジアンに変換
    }
    else
    {
      // (1, -1.5, -30°)
      pose_msg.x = 1.0;
      pose_msg.y = -1.5;
      pose_msg.theta = -30.0 * M_PI / 180.0; // ラジアンに変換
    }

    // ターゲットPose2Dをパブリッシュ
    pose_publisher_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "Pose2Dをパブリッシュ: x=%.2f, y=%.2f, theta=%.2f rad",
                pose_msg.x, pose_msg.y, pose_msg.theta);

    // トグルフラグを反転
    toggle_pose_ = !toggle_pose_;
  }
} // namespace impact_point_estimator