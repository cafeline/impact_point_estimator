#pragma once

#include <vector>
#include <deque>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include "impact_point_estimator/trajectory_filter.hpp"
#include "impact_point_estimator/trajectory_predictor.hpp"

namespace impact_point_estimator
{

  class ImpactPointEstimatorCore
  {
  public:
    ImpactPointEstimatorCore(double v_min, double v_max,
                             const std::vector<double> &expected_direction,
                             double theta_max_deg,
                             int curve_points_num,
                             double lidar_to_target_x,
                             double lidar_to_target_y,
                             double lidar_to_target_z,
                             double minDistance = 0.1);

    // 入力された点（およびそのタイムスタンプ）を処理し、十分なデータがあれば予測結果を返す
    // 戻り値が true なら predictionResult に予測結果が格納される
    bool processPoint(const geometry_msgs::msg::Point &point, double timestamp, PredictionResult &predictionResult);

    void clearData();

  private:
    TrajectoryFilter filter_;
    TrajectoryPredictor predictor_;
    int curve_points_num_;
    double lidar_to_target_x_;
    double lidar_to_target_y_;
    double lidar_to_target_z_;
    double minDistance_;

    std::vector<geometry_msgs::msg::Point> points_;
    std::vector<double> timestamps_;
    std::deque<geometry_msgs::msg::Point> recent_points_;

    // 内部で点を追加（フィルタチェックなど）
    bool addPoint(const geometry_msgs::msg::Point &point, double timestamp);
  };

} // namespace impact_point_estimator
