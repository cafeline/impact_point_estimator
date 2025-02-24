#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>
#include <vector>

#include "impact_point_estimator/trajectory_filter.hpp"
#include "impact_point_estimator/trajectory_predictor.hpp"

namespace impact_point_estimator
{

  class ImpactPointEstimatorNode : public rclcpp::Node
  {
  public:
    explicit ImpactPointEstimatorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ImpactPointEstimatorNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    // コールバック
    void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    // ユーティリティ：PointStamped, Marker 作成
    geometry_msgs::msg::PointStamped createPointStamped(double x, double y, double z) const;
    visualization_msgs::msg::Marker createMarker(const std::string &ns, int id, int type,
                                                 double scale_x, double scale_y, double scale_z,
                                                 double r, double g, double b, double a) const;

    // パブリッシャーへの送信ヘルパー
    void publishEstimatedImpact(const PredictionResult &result);
    void publishMarker(const visualization_msgs::msg::Marker &marker);
    void publishTargetPose(const geometry_msgs::msg::Point &point);
    void publishMotorPosition(double angle_rad);

    // タイマー処理
    void scheduleMotorPosition(double delay);
    void scheduleStandby(double delay);

    void processTimeout(const visualization_msgs::msg::Marker::SharedPtr msg,
                          std::chrono::steady_clock::time_point now, double dt);
    bool processIncomingPoint(const geometry_msgs::msg::Point &point,
                                std::chrono::steady_clock::time_point now);
    void processPrediction();

    // データ管理
    void clearData();

    // ROS インターフェース
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr standby_timer_;

    // 状態管理
    bool is_predicting_;
    std::vector<geometry_msgs::msg::Point> points_;
    std::deque<geometry_msgs::msg::Point> recent_points_;
    std::vector<double> timestamps_;
    std::chrono::steady_clock::time_point last_point_time_;
    std::chrono::steady_clock::time_point start_time_;

    // パラメータ
    double motor_pos_;
    double offset_time_;
    int curve_points_num_;
    double standby_pose_x_;
    double standby_pose_y_;
    double reroad_;
    double lidar_to_target_x_;
    double lidar_to_target_y_;
    double lidar_to_target_z_;
    double v_min_;
    double v_max_;
    std::vector<double> expected_direction_;
    double theta_max_deg_;
    double first_goal_x_;
    double standby_delay_;

    // フィルタと予測クラス
    TrajectoryFilter filter_;
    TrajectoryPredictor predictor_;
  };

} // namespace impact_point_estimator
