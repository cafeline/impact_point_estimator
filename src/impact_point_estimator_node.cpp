#include "impact_point_estimator/impact_point_estimator_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace impact_point_estimator
{

  ImpactPointEstimatorNode::ImpactPointEstimatorNode(const rclcpp::NodeOptions &options)
      : ImpactPointEstimatorNode("impact_point_estimator", options)
  {
  }

  ImpactPointEstimatorNode::ImpactPointEstimatorNode(const std::string &node_name, const rclcpp::NodeOptions &options)
      : Node(node_name, options)
  {
    // ROSパラメータの取得
    motor_pos_ = this->get_parameter("motor_pos").as_double();
    offset_time_ = this->get_parameter("offset_time").as_double();
    curve_points_num_ = this->get_parameter("curve_points_num").as_int();
    standby_pose_x_ = this->get_parameter("standby_pose_x").as_double();
    standby_pose_y_ = this->get_parameter("standby_pose_y").as_double();
    reroad_ = this->get_parameter("reroad").as_double();
    lidar_to_target_x_ = this->get_parameter("lidar_to_target_x").as_double();
    lidar_to_target_y_ = this->get_parameter("lidar_to_target_y").as_double();
    lidar_to_target_z_ = this->get_parameter("lidar_to_target_z").as_double();
    v_min_ = this->get_parameter("V_min").as_double();
    v_max_ = this->get_parameter("V_max").as_double();
    expected_direction_ = this->get_parameter("expected_direction").as_double_array();
    theta_max_deg_ = this->get_parameter("theta_max_deg").as_double();
    first_goal_x_ = this->get_parameter("first_goal_x").as_double();
    standby_delay_ = this->get_parameter("standby_delay").as_double();

    core_ = std::make_unique<ImpactPointEstimatorCore>(v_min_, v_max_, expected_direction_,
                                                       theta_max_deg_, curve_points_num_, lidar_to_target_x_, lidar_to_target_y_, lidar_to_target_z_);

    predictor_ = std::make_unique<TrajectoryPredictor>();

    marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "tennis_ball", 10, std::bind(&ImpactPointEstimatorNode::markerCallback, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fitted_curve", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_pose", 10);
    motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor/pos", 10);

    start_time_ = std::chrono::steady_clock::now();
    last_time_ = start_time_;

    RCLCPP_INFO(this->get_logger(), "Initialized ImpactPointEstimatorNode");
  }

  geometry_msgs::msg::PointStamped ImpactPointEstimatorNode::createPointStamped(double x, double y, double z) const
  {
    geometry_msgs::msg::PointStamped stamp;
    stamp.header.frame_id = "map";
    stamp.header.stamp = this->now();
    stamp.point.x = x;
    stamp.point.y = y;
    stamp.point.z = z;
    return stamp;
  }

  visualization_msgs::msg::Marker ImpactPointEstimatorNode::createMarker(const std::string &ns, int id, int type,
                                                                         double scale_x, double scale_y, double scale_z,
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

  void ImpactPointEstimatorNode::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    // 現在時刻取得
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();

    // タイムアウト判定：一定時間以上経過している場合は前回シーケンスを破棄し、start_time_をリセット
    if (dt > 0.35)
    {
      core_->clearData();
      start_time_ = now; // 次の最初のボールのタイムスタンプを0.0にするためリセット
    }
    last_time_ = now;

    // 現在時刻との差からタイムスタンプを計算（タイムアウト後は0.0になる）
    double timestamp = std::chrono::duration<double>(now - start_time_).count();

    // コア処理に点を渡す
    PredictionResult result;
    bool predictionAvailable = core_->processPoint(msg->pose.position, timestamp, result);
    if (predictionAvailable && result.success)
    {
      publishEstimatedImpact(result);
      double adjusted_time = result.impact_time - static_cast<double>(curve_points_num_) * 0.1 + offset_time_;
      scheduleMotorPosition(adjusted_time);
    }
  }

  void ImpactPointEstimatorNode::publishEstimatedImpact(const PredictionResult &result)
  {
    // まず軌道の可視化は常に行う
    auto trajectory = predictor_->generateTrajectoryPoints(result.x0, result.y0, result.z0,
                                                           result.vx, result.vy, result.vz, result.impact_time);
    auto marker = createMarker("fitted_curve", 0, visualization_msgs::msg::Marker::LINE_STRIP,
                               0.02, 0.02, 0.02, 1.0, 0.0, 0.0, 1.0);
    marker.points = trajectory;
    publishMarker(marker);

    // 弾道の予測結果のx方向の速度が負の場合のみターゲットポーズを発行
    if (result.vx < 0)
    {
      auto target_pose = createPointStamped(result.x_impact, result.y_impact, 0.0);
      publishTargetPose(target_pose.point);
    }
  }

  void ImpactPointEstimatorNode::publishMarker(const visualization_msgs::msg::Marker &marker)
  {
    marker_pub_->publish(marker);
  }

  void ImpactPointEstimatorNode::publishTargetPose(const geometry_msgs::msg::Point &point)
  {
    auto stamped = createPointStamped(point.x, point.y, point.z);
    pose_pub_->publish(stamped);
    RCLCPP_INFO(this->get_logger(), "Published target pose: (%.2f, %.2f)", point.x, point.y);
  }

  void ImpactPointEstimatorNode::publishMotorPosition(double angle_rad)
  {
    std_msgs::msg::Float64 msg;
    msg.data = angle_rad;
    motor_pub_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Motor position published: %.2f", angle_rad);
  }

  void ImpactPointEstimatorNode::scheduleMotorPosition(double delay)
  {
    if (delay > 0)
    {
      auto delay_ms = std::chrono::milliseconds(static_cast<int>(delay * 1000));
      timer_ = this->create_wall_timer(delay_ms, [this]()
                                       {
        publishMotorPosition(motor_pos_);
        timer_->cancel(); });
    }
    else
    {
      publishMotorPosition(motor_pos_);
    }
  }

  void ImpactPointEstimatorNode::scheduleStandby(double delay)
  {
    if (delay > 0)
    {
      auto delay_ms = std::chrono::milliseconds(static_cast<int>(delay * 1000));
      standby_timer_ = this->create_wall_timer(delay_ms, [this]()
                                               {
        auto standby_pose = createPointStamped(standby_pose_x_, standby_pose_y_, 0.0);
        publishTargetPose(standby_pose.point);
        std_msgs::msg::Float64 motor_msg;
        motor_msg.data = reroad_;
        motor_pub_->publish(motor_msg);
        standby_timer_->cancel(); });
    }
    else
    {
      auto standby_pose = createPointStamped(standby_pose_x_, standby_pose_y_, 0.0);
      publishTargetPose(standby_pose.point);
      std_msgs::msg::Float64 motor_msg;
      motor_msg.data = reroad_;
      motor_pub_->publish(motor_msg);
    }
  }

} // namespace impact_point_estimator
