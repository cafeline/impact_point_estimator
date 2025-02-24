#include "impact_point_estimator/impact_point_estimator.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace impact_point_estimator
{
  geometry_msgs::msg::PointStamped ImpactPointEstimator::createPointStamped(double x, double y, double z) const
  {
    geometry_msgs::msg::PointStamped stamp;
    stamp.header.frame_id = "map";
    stamp.header.stamp = this->now();
    stamp.point.x = x;
    stamp.point.y = y;
    stamp.point.z = z;
    return stamp;
  }

  visualization_msgs::msg::Marker ImpactPointEstimator::createMarker(const std::string &ns,
                                                                     int id,
                                                                     int type,
                                                                     double scale_x,
                                                                     double scale_y,
                                                                     double scale_z,
                                                                     double r, double g, double b, double a) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
  }

  void ImpactPointEstimator::handleDtTimeout(const geometry_msgs::msg::Point &point, const std::chrono::steady_clock::time_point &now)
  {
    RCLCPP_WARN(this->get_logger(), "dt exceeds threshold. Clearing data.");
    clear_data();
    last_point_time_ = now;
    if (point.z > lidar_to_target_z_)
    {
      points_.emplace_back(point);
      prediction_.add_timestamp(0.0);
    }
  }

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
    standby_delay_ = this->get_parameter("standby_delay").as_double();

    // サブスクライバーの設定
    subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "tennis_ball", 10, std::bind(&ImpactPointEstimator::listener_callback, this, std::placeholders::_1));

    // パブリッシャーの設定
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_curve", 10);
    points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fitted_points", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_pose", 10);
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
    auto start = std::chrono::high_resolution_clock::now();

    geometry_msgs::msg::Point point = msg->pose.position;
    // 現在時刻を取得
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_point_time_).count();

    if (!prediction_.is_start_time_initialized())
    {
      prediction_.set_start_time(now);
    }

    double time_stamp = prediction_.calculate_relative_time(now);

    if (dt > 0.35)
    {
      handleDtTimeout(point, now);
      return;
    }

    last_point_time_ = now;

    if (!filter_.check_point_validity(point, points_, recent_points_, lidar_to_target_z_))
    {
      RCLCPP_WARN(this->get_logger(), "point invalidity");
      return;
    }
    points_.emplace_back(point);
    prediction_.add_timestamp(time_stamp + 0.1);

    if (points_.size() == 2)
    {
      if (!filter_.validate_vel_and_direction(points_[0], points_[1], dt))
      {
        RCLCPP_WARN(this->get_logger(), "Two points rejected by BallFilter");
        clear_data();
        return;
      }
      schedule_standby_and_reroad(standby_delay_);
    }

    if (points_.size() >= static_cast<size_t>(curve_points_num_))
    {
      prediction_.process_points(points_, lidar_to_target_x_, lidar_to_target_y_, lidar_to_target_z_,
                                 [this](const PredictionResult &result)
                                 {
                                   if (result.success)
                                   {
                                     publish_estimated_impact(result.impact_time, result.x_impact, result.y_impact,
                                                              result.x0, result.y0, result.z0, result.vx, result.vy, result.vz);
                                     double adjusted_impact_time = result.impact_time - points_.size() * 0.1 + offset_time_;
                                     RCLCPP_INFO(this->get_logger(), "adjusted_impact_time: %.2f", adjusted_impact_time);
                                     schedule_motor_position(adjusted_impact_time);
                                     publish_points_marker();
                                   }
                                 });
    }
    RCLCPP_INFO(this->get_logger(), "exec time: %ld ms",
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count());
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

      auto target_pose = createPointStamped(target_x, target_y, 0.0);
      pose_publisher_->publish(target_pose);
      RCLCPP_INFO(this->get_logger(), "Published target_pose: x=%.2f, y=%.2f", target_x, target_y);

      publish_two_ball_marker(points[0], points[1]);

      std::vector<geometry_msgs::msg::Point> trajectory_points{points[0], points[1]};
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
    prediction_.reset_start_time();
  }

  void ImpactPointEstimator::publish_estimated_impact(double impact_time, double x_impact, double y_impact,
                                                      double x0, double y0, double z0,
                                                      double vx, double vy, double vz)
  {
    RCLCPP_INFO(this->get_logger(), "着弾時間: %.2f s, 着弾地点: (%.3f, %.3f), height=%.3f", impact_time, x_impact, y_impact, lidar_to_target_z_);

    auto final_pose = createPointStamped(x_impact, y_impact, 0.0);
    publish_final_pose(final_pose.point);

    auto trajectory_points = prediction_.generate_trajectory_points(x0, y0, z0, vx, vy, vz, impact_time);
    publish_curve_marker(trajectory_points);
  }

  void ImpactPointEstimator::publish_curve_marker(const std::vector<geometry_msgs::msg::Point> &curve_points)
  {
    auto marker = createMarker("fitted_curve", 0, visualization_msgs::msg::Marker::LINE_STRIP,
                               0.02, 0.02, 0.02,
                               1.0, 0.0, 0.0, 1.0);
    marker.points = curve_points;
    publisher_->publish(marker);
  }

  void ImpactPointEstimator::publish_points_marker()
  {
    auto marker = createMarker("original_points", 1, visualization_msgs::msg::Marker::SPHERE_LIST,
                               0.07, 0.07, 0.07,
                               0.0, 1.0, 0.5, 1.0);
    marker.lifetime = rclcpp::Duration(0, 0);
    marker.points = points_;
    points_publisher_->publish(marker);
  }

  void ImpactPointEstimator::publish_final_pose(const geometry_msgs::msg::Point &final_point)
  {
    auto target_pose = createPointStamped(final_point.x, final_point.y, 0.0);
    pose_publisher_->publish(target_pose);
    RCLCPP_INFO(this->get_logger(), "Published target_pose: x=%.2f, y=%.2f z=%.2f",
                target_pose.point.x, target_pose.point.y, target_pose.point.z);
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

  void ImpactPointEstimator::schedule_standby_and_reroad(double delay)
  {
    if (delay > 0.0)
    {
      auto delay_ms = std::chrono::milliseconds(static_cast<int>(delay * 1000));
      standby_timer_ = this->create_wall_timer(
          delay_ms,
          [this]()
          {
            auto standby_pose = createPointStamped(standby_pose_x_, standby_pose_y_, 0.0);
            pose_publisher_->publish(standby_pose);
            std_msgs::msg::Float64 motor_msg;
            motor_msg.data = reroad_;
            motor_pos_publisher_->publish(motor_msg);
            standby_timer_->cancel();
          });
    }
    else
    {
      auto standby_pose = createPointStamped(standby_pose_x_, standby_pose_y_, 0.0);
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
    std_msgs::msg::Float64 message;
    message.data = angle_rad;
    motor_pos_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published motor_pos_: %f rad", angle_rad);
  }

  void ImpactPointEstimator::publish_two_ball_marker(const geometry_msgs::msg::Point &point1, const geometry_msgs::msg::Point &point2)
  {
    auto ball_marker = createMarker("ball_markers", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
                                    0.1, 0.1, 0.1,
                                    1.0, 1.0, 0.0, 1.0);
    ball_marker.points.push_back(point1);
    ball_marker.points.push_back(point2);
    publisher_->publish(ball_marker);
  }

  void ImpactPointEstimator::publish_linear_trajectory_marker(const std::vector<geometry_msgs::msg::Point> &trajectory_points)
  {
    auto trajectory_marker = createMarker("linear_trajectory", 1, visualization_msgs::msg::Marker::LINE_STRIP,
                                          0.05, 0.05, 0.05,
                                          0.0, 0.0, 1.0, 0.5);
    trajectory_marker.points = trajectory_points;
    publisher_->publish(trajectory_marker);
  }

  void ImpactPointEstimator::publish_three_points_curve(const std::vector<geometry_msgs::msg::Point> &curve_points)
  {
    auto marker = createMarker("cubic_curve", 0, visualization_msgs::msg::Marker::LINE_STRIP,
                               0.05, 0.05, 0.05,
                               0.5, 0.0, 0.5, 0.5);
    marker.points = curve_points;
    publisher_->publish(marker);
  }
} // namespace impact_point_estimator
