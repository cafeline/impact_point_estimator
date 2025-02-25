#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>

#include "impact_point_estimator/trajectory_filter.hpp"
#include "impact_point_estimator/trajectory_predictor.hpp"
#include "impact_point_estimator/impact_point_estimator_core.hpp"

namespace impact_point_estimator
{

  class ImpactPointEstimatorNode : public rclcpp::Node
  {
  public:
    explicit ImpactPointEstimatorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ImpactPointEstimatorNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    geometry_msgs::msg::PointStamped createPointStamped(double x, double y, double z) const;
    visualization_msgs::msg::Marker createMarker(const std::string &ns, int id, int type,
                                                 double scale_x, double scale_y, double scale_z,
                                                 double r, double g, double b, double a) const;

    void publishEstimatedImpact(const PredictionResult &result);
    void publishMarker(const visualization_msgs::msg::Marker &marker);
    void publishTargetPose(const geometry_msgs::msg::Point &point);
    void publishMotorPosition(double angle_rad);
    void scheduleMotorPosition(double delay);
    void scheduleStandby(double delay);

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr standby_timer_;

    std::unique_ptr<ImpactPointEstimatorCore> core_;
    std::unique_ptr<TrajectoryPredictor> predictor_;

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

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_time_;
  };

} // namespace impact_point_estimator
