#include "impact_point_estimator/impact_point_estimator_core.hpp"
#include <rclcpp/rclcpp.hpp>

namespace impact_point_estimator
{

  ImpactPointEstimatorCore::ImpactPointEstimatorCore(double v_min, double v_max,
                                                     const std::vector<double> &expected_direction,
                                                     double theta_max_deg,
                                                     int curve_points_num,
                                                     double lidar_to_target_x,
                                                     double lidar_to_target_y,
                                                     double lidar_to_target_z,
                                                     double minDistance)
      : filter_(v_min, v_max, expected_direction, theta_max_deg),
        curve_points_num_(curve_points_num),
        lidar_to_target_x_(lidar_to_target_x),
        lidar_to_target_y_(lidar_to_target_y),
        lidar_to_target_z_(lidar_to_target_z),
        minDistance_(minDistance)
  {
  }

  bool ImpactPointEstimatorCore::addPoint(const geometry_msgs::msg::Point &point, double timestamp)
  {
    geometry_msgs::msg::Point *lastValid = points_.empty() ? nullptr : &points_.back();
    if (!filter_.isPointValid(point, recent_points_, lastValid, minDistance_, lidar_to_target_z_))
    {
      RCLCPP_WARN(rclcpp::get_logger("ImpactPointEstimatorCore"), "Point validation failed in core.");
      return false;
    }
    points_.push_back(point);
    timestamps_.push_back(timestamp);
    recent_points_.push_back(point);
    if (recent_points_.size() > 5)
      recent_points_.pop_front();
    return true;
  }

  bool ImpactPointEstimatorCore::processPoint(const geometry_msgs::msg::Point &point, double timestamp, PredictionResult &predictionResult)
  {
    if (!addPoint(point, timestamp))
      return false;

    if (points_.size() >= static_cast<size_t>(curve_points_num_))
    {
      // 予測処理を呼び出し、結果を同期的に predictionResult に格納する
      for (int i = 0; i < points_.size(); i++)
      {
        RCLCPP_INFO(rclcpp::get_logger("ImpactPointEstimatorCore"), "points_[%d]: x=%f, y=%f, z=%f", i, points_[i].x, points_[i].y, points_[i].z);
        RCLCPP_INFO(rclcpp::get_logger("ImpactPointEstimatorCore"), "timestamp: %f", timestamps_[i]);
      }
      predictor_.predictTrajectory(points_, timestamps_,
                                   lidar_to_target_x_, lidar_to_target_y_, lidar_to_target_z_,
                                   [&predictionResult](const PredictionResult &result)
                                   {
                                     predictionResult = result;
                                   });
      return true;
    }
    return false;
  }

  void ImpactPointEstimatorCore::clearData()
  {
    points_.clear();
    timestamps_.clear();
    recent_points_.clear();
    RCLCPP_INFO(rclcpp::get_logger("ImpactPointEstimatorCore"), "################# Data cleared #################");
  }

} // namespace impact_point_estimator
