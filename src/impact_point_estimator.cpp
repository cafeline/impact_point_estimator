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
        filter_(0.0, 0.0, {1.0, 0.0, -1.0}, 30.0),
        prediction_()
  {
    RCLCPP_INFO(this->get_logger(), "impact_point_estimatorの初期化");

    motor_pos_ = this->get_parameter("motor_pos").as_double();
    offset_time_ = this->get_parameter("offset_time").as_double();
    curve_points_num_ = this->get_parameter("curve_points_num").as_int();
    standby_pose_x_ = this->get_parameter("standby_pose_x").as_double();
    standby_pose_y_ = this->get_parameter("standby_pose_y").as_double();
    reroad_ = this->get_parameter("reroad").as_double();
    lidar_to_target_x_ = this->get_parameter("lidar_to_target_x").as_double();
    lidar_to_target_y_ = this->get_parameter("lidar_to_target_y").as_double();
    lidar_to_target_z_ = this->get_parameter("lidar_to_target_z").as_double();
    V_min_ = this->get_parameter("V_min").as_double();
    V_max_ = this->get_parameter("V_max").as_double();
    expected_direction_ = this->get_parameter("expected_direction").as_double_array();
    theta_max_deg_ = this->get_parameter("theta_max_deg").as_double();
    first_goal_x_ = this->get_parameter("first_goal_x").as_double();
    // サブスクライバーの設定
    subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "tennis_ball", 10, std::bind(&ImpactPointEstimator::listener_callback, this, std::placeholders::_1));

    // パブリッシャーの設定
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_curve", 10);
    points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_points", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/target_pose", 10);
    motor_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("motor/pos", 10);

    filter_ = Filter(V_min_, V_max_, expected_direction_, theta_max_deg_);

    last_point_time_ = std::chrono::steady_clock::now();
  }

  void ImpactPointEstimator::listener_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (!is_predicting_)
    {
      return;
    }

    geometry_msgs::msg::Point point = msg->pose.position;
    // 現在時刻を取得
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_point_time_).count();

    // RCLCPP_INFO(this->get_logger(), "dt %f, point x: %.2f, y: %.2f, z: %.2f", dt, point.x, point.y, point.z);
    if (!prediction_.is_start_time_initialized())
    {
      prediction_.set_start_time(now);
    }

    // 相対時間を計算
    double time_stamp = prediction_.calculate_relative_time(now);
    // RCLCPP_INFO(this->get_logger(), "dt: %.2f", dt);

    if (dt > 0.35)
    {
      RCLCPP_WARN(this->get_logger(), "dt = %.2f > 0.35 :clear_data", dt);
      clear_data();
      last_point_time_ = now;
      if (point.z > lidar_to_target_z_)
      {
        points_.emplace_back(point);
        time_stamp = -0.1;
        prediction_.add_timestamp(time_stamp);
      }
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "points_size: %d", points_.size());

    last_point_time_ = now;

    // ポイントとdtを検証・追加
    if (!filter_.check_point_validity(point, points_, recent_points_, lidar_to_target_z_))
    {
      RCLCPP_WARN(this->get_logger(), "point invalidity");
      return;
    }
    points_.emplace_back(point);
    prediction_.add_timestamp(time_stamp);

    if (points_.size() == 2)
    {
      if (!filter_.validate_vel_and_direction(points_[0], points_[1], dt))
      {
        RCLCPP_WARN(this->get_logger(), "Two points rejected by BallFilter");
        clear_data();
        return;
      }
      process_two_points(points_);
      // 3秒後に standby_pose と reroad_ をpublish
      double standby_delay = 3.0;
      schedule_standby_and_reroad(standby_delay);
    }

    if (points_.size() >= static_cast<size_t>(curve_points_num_))
    {
      // RCLCPP_INFO(this->get_logger(), "##################################################");
      // for (int i = 0; i < points_.size(); i++)
      // {
      //   if (i > 0)
      //   {
      //     double dt = prediction_.timestamps_[i] - prediction_.timestamps_[i - 1];
      //     double vx = (points_[i].x - points_[i - 1].x) / dt;
      //     double vy = (points_[i].y - points_[i - 1].y) / dt;
      //     double vz = (points_[i].z - points_[i - 1].z) / dt;
      //     double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
      //     RCLCPP_INFO(this->get_logger(), "timestamp: %.2f", prediction_.timestamps_[i]);
      //     RCLCPP_INFO(this->get_logger(), "point x: %.2f, y: %.2f, z: %.2f", points_[i].x, points_[i].y, points_[i].z);
      //     RCLCPP_INFO(this->get_logger(), "speed: %.2f", speed);
      //   }
      //   else
      //   {
      //     RCLCPP_INFO(this->get_logger(), "0 timestamp: %.2f", prediction_.timestamps_[i]);
      //     RCLCPP_INFO(this->get_logger(), "0 point x: %.2f, y: %.2f, z: %.2f", points_[i].x, points_[i].y, points_[i].z);
      //   }
      // }
      // RCLCPP_INFO(this->get_logger(), "##################################################");

      prediction_.process_points(points_, lidar_to_target_x_, lidar_to_target_y_, lidar_to_target_z_, [this](const PredictionResult &result)
                                 {
        if (result.success)
        {
          // 予測が成功したらすぐに着弾地点をpublish
          publish_estimated_impact(result.impact_time, result.x_impact, result.y_impact, result.x0, result.y0, result.z0, result.vx, result.vy, result.vz);
          // impact_time 後に motor_pos_ をパブリッシュ
          schedule_motor_position(result.impact_time + offset_time_);

          publish_points_marker();
        }
        pause_processing(); });
      clear_data();
    }
  }

  void ImpactPointEstimator::process_two_points(const std::vector<geometry_msgs::msg::Point> &points)
  {
    double x1 = points[0].x;
    double y1 = points[0].y;
    double x2 = points[1].x;
    double y2 = points[1].y;
    double target_x = first_goal_x_;
    double target_y;

    if (x2 - x1 != 0)
    {
      double slope = (y2 - y1) / (x2 - x1);
      target_y = y1 + slope * (target_x - x1);

      // 目標姿勢を作成してパブリッシュ
      geometry_msgs::msg::Pose2D target_pose;
      target_pose.x = target_x;
      target_pose.y = target_y;
      target_pose.theta = 0.0;
      pose_publisher_->publish(target_pose);
      RCLCPP_INFO(this->get_logger(), "Published target_pose: x=%.2f, y=%.2f", target_x, target_y);

      // ボールマーカーをパブリッシュ
      publish_two_ball_marker(points[0], points[1]);

      // 2点間を線で結ぶ軌道を作成してパブリッシュ
      std::vector<geometry_msgs::msg::Point> trajectory_points;
      trajectory_points.emplace_back(points[0]);
      trajectory_points.emplace_back(points[1]);
      publish_linear_trajectory_marker(trajectory_points);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "入力された2点が垂直です。目標位置を計算できません。");
    }
  }

  void ImpactPointEstimator::clear_data()
  {
    points_.clear();
    recent_points_.clear();
    prediction_.timestamps_.clear();
    // Prediction内部の開始時刻フラグをリセット
    prediction_.reset_start_time();
  }

  void ImpactPointEstimator::publish_estimated_impact(
      double impact_time, double x_impact, double y_impact,
      double x0, double y0, double z0,
      double vx, double vy, double vz)
  {
    RCLCPP_INFO(this->get_logger(), "着弾時間: %.2f s, 着弾地点: (%.2f, %.2f), height=%.2f", impact_time, x_impact, y_impact, lidar_to_target_z_);

    // 可視化用に軌道をプロット
    std::vector<geometry_msgs::msg::Point> trajectory_points = prediction_.generate_trajectory_points(x0, y0, z0, vx, vy, vz, impact_time);
    publish_curve_marker(trajectory_points);

    // 着弾地点をすぐにpublish
    geometry_msgs::msg::Point final_point;
    final_point.x = x_impact;
    final_point.y = y_impact;
    final_point.z = lidar_to_target_z_;
    publish_final_pose(final_point);
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
    RCLCPP_INFO(this->get_logger(), "Published target_pose: x=%.2f, y=%.2f (height=%.2f), theta=%.2f",
                target_pose.x, target_pose.y, final_point.z, target_pose.theta);
  }

  void ImpactPointEstimator::schedule_motor_position(double delay)
  {
    if (delay > 0.0)
    {
      auto delay_ms = std::chrono::milliseconds(static_cast<int>(delay * 1000));
      timer_ = this->create_wall_timer(
          delay_ms,
          [this]()
          {
            publish_motor_pos(motor_pos_);
            timer_->cancel();
          });
    }
    else
    {
      publish_motor_pos(motor_pos_);
    }
  }

  // impact_time + 3秒後にstandby_poseとreroad_をpublishする
  void ImpactPointEstimator::schedule_standby_and_reroad(double delay)
  {
    if (delay > 0.0)
    {
      auto delay_ms = std::chrono::milliseconds(static_cast<int>(delay * 1000));
      standby_timer_ = this->create_wall_timer(
          delay_ms,
          [this]()
          {
            // standby_poseのpublish
            geometry_msgs::msg::Pose2D standby_pose;
            standby_pose.x = standby_pose_x_;
            standby_pose.y = standby_pose_y_;
            standby_pose.theta = 0.0;
            pose_publisher_->publish(standby_pose);
            // reroad_をpublish
            std_msgs::msg::Float64 motor_msg;
            motor_msg.data = reroad_;
            motor_pos_publisher_->publish(motor_msg);

            standby_timer_->cancel();
          });
    }
    else
    {
      geometry_msgs::msg::Pose2D standby_pose;
      standby_pose.x = standby_pose_x_;
      standby_pose.y = standby_pose_y_;
      standby_pose.theta = 0.0;
      pose_publisher_->publish(standby_pose);

      std_msgs::msg::Float64 motor_msg;
      motor_msg.data = reroad_;
      motor_pos_publisher_->publish(motor_msg);
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
    is_predicting_ = true;
  }

  void ImpactPointEstimator::publish_motor_pos(double angle_rad)
  {
    auto message = std_msgs::msg::Float64();
    message.data = angle_rad;
    motor_pos_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published motor_pos_: %f rad", angle_rad);
  }

  void ImpactPointEstimator::publish_two_ball_marker(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2)
  {
    visualization_msgs::msg::Marker ball_marker;
    ball_marker.header.frame_id = "map";
    ball_marker.header.stamp = this->get_clock()->now();
    ball_marker.ns = "ball_markers";
    ball_marker.id = 0;
    ball_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    ball_marker.action = visualization_msgs::msg::Marker::ADD;

    // ボールのサイズを設定
    ball_marker.scale.x = 0.1;
    ball_marker.scale.y = 0.1;
    ball_marker.scale.z = 0.1;

    // ボールの色を設定（黄色）
    ball_marker.color.r = 1.0;
    ball_marker.color.g = 1.0;
    ball_marker.color.b = 0.0;
    ball_marker.color.a = 1.0;

    // ボールの位置を追加
    ball_marker.points.emplace_back(point1);
    ball_marker.points.emplace_back(point2);

    // マーカーをパブリッシュ
    publisher_->publish(ball_marker);
  }

  void ImpactPointEstimator::publish_linear_trajectory_marker(const std::vector<geometry_msgs::msg::Point> &trajectory_points)
  {
    visualization_msgs::msg::Marker trajectory_marker;
    trajectory_marker.header.frame_id = "map";
    trajectory_marker.header.stamp = this->get_clock()->now();
    trajectory_marker.ns = "linear_trajectory";
    trajectory_marker.id = 1;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.scale.x = 0.05; // 線の太さ

    // 色の設定（青色）
    trajectory_marker.color.r = 0.0;
    trajectory_marker.color.g = 0.0;
    trajectory_marker.color.b = 1.0;
    trajectory_marker.color.a = 1.0;

    trajectory_marker.points = trajectory_points;
    publisher_->publish(trajectory_marker);
  }

} // namespace impact_point_estimator
