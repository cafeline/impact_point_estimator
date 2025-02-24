#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <functional>
#include <chrono>
#include <eigen3/Eigen/Dense>

namespace impact_point_estimator
{

  struct PredictionResult
  {
    bool success;
    double impact_time;
    double x_impact;
    double y_impact;
    double x0;
    double y0;
    double z0;
    double vx;
    double vy;
    double vz;
  };

  class TrajectoryPredictor
  {
  public:
    using PredictionCallback = std::function<void(const PredictionResult &)>;

    // 複数点とタイムスタンプから弾道予測を行う
    void predictTrajectory(const std::vector<geometry_msgs::msg::Point> &points,
                           const std::vector<double> &timestamps,
                           double lidar_to_target_x,
                           double lidar_to_target_y,
                           double lidar_to_target_z,
                           PredictionCallback callback);

    // 弾道モデルから軌道点列を生成する
    std::vector<geometry_msgs::msg::Point> generateTrajectoryPoints(double x0, double y0, double z0,
                                                                    double vx, double vy, double vz,
                                                                    double impact_time) const;

  private:
    // // 立方曲線フィッティング（最小二乗法）
    // std::vector<geometry_msgs::msg::Point> fitCubicCurve(const std::vector<geometry_msgs::msg::Point> &points,
    //                                                      Eigen::VectorXd &coeffs_x,
    //                                                      Eigen::VectorXd &coeffs_y,
    //                                                      Eigen::VectorXd &coeffs_z);

    // 弾道モデル（直線+重力項）によるフィッティング
    bool fitBallisticTrajectory(const std::vector<geometry_msgs::msg::Point> &points,
                                const std::vector<double> &times,
                                double &x0, double &y0, double &z0,
                                double &vx, double &vy, double &vz);

    // 目標高さにおける着弾時刻・位置を計算
    bool calculateImpactPoint(double lidar_to_target_x,
                              double lidar_to_target_y,
                              double lidar_to_target_z,
                              double z0, double vz,
                              double x0, double y0,
                              double vx, double vy,
                              double &impact_time,
                              double &x_impact,
                              double &y_impact);

    double evaluateCubic(const Eigen::VectorXd &coeffs, double t) const;
  };

} // namespace impact_point_estimator
