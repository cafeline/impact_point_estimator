#include "impact_point_estimator/trajectory_predictor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <random>
#include <algorithm>

namespace impact_point_estimator
{
  void TrajectoryPredictor::predictTrajectory(const std::vector<geometry_msgs::msg::Point> &points,
                                              const std::vector<double> &timestamps,
                                              double lidar_to_target_x,
                                              double lidar_to_target_y,
                                              double lidar_to_target_z,
                                              PredictionCallback callback)
  {
    double x0, y0, z0, vx, vy, vz;
    if (!fitBallisticTrajectory(points, timestamps, x0, y0, z0, vx, vy, vz))
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "Ballistic trajectory fitting failed.");
      callback({false, -1, 0, 0, 0, 0, 0, 0, 0, 0});
      return;
    }
    double impact_time, x_impact, y_impact;
    if (!calculateImpactPoint(lidar_to_target_x, lidar_to_target_y, lidar_to_target_z,
                              z0, vz, x0, y0, vx, vy, impact_time, x_impact, y_impact))
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "Impact point calculation failed.");
      callback({false, -1, 0, 0, 0, 0, 0, 0, 0, 0});
      return;
    }
    callback({true, impact_time, x_impact, y_impact, x0, y0, z0, vx, vy, vz});
  }

  bool TrajectoryPredictor::fitBallisticTrajectory(const std::vector<geometry_msgs::msg::Point> &points,
                                                   const std::vector<double> &times,
                                                   double &x0, double &y0, double &z0,
                                                   double &vx, double &vy, double &vz)
  {
    if (points.size() < 2)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "Not enough data for ballistic fit.");
      return false;
    }
    size_t N = points.size();
    double g = 9.81;
    Eigen::MatrixXd A_x(N, 2), A_y(N, 2), A_z(N, 2);
    Eigen::VectorXd X(N), Y(N), Z_mod(N);
    for (size_t i = 0; i < N; ++i)
    {
      double t = times[i];
      A_x(i, 0) = 1.0;
      A_x(i, 1) = t;
      A_y(i, 0) = 1.0;
      A_y(i, 1) = t;
      A_z(i, 0) = 1.0;
      A_z(i, 1) = t;
      X(i) = points[i].x;
      Y(i) = points[i].y;
      Z_mod(i) = points[i].z + 0.5 * g * t * t;
    }
    Eigen::Vector2d p_x = A_x.colPivHouseholderQr().solve(X);
    Eigen::Vector2d p_y = A_y.colPivHouseholderQr().solve(Y);
    Eigen::Vector2d p_z = A_z.colPivHouseholderQr().solve(Z_mod);
    x0 = p_x(0);
    vx = p_x(1);
    y0 = p_y(0);
    vy = p_y(1);
    z0 = p_z(0);
    vz = p_z(1);
    if (std::isnan(x0) || std::isnan(y0) || std::isnan(z0) ||
        std::isnan(vx) || std::isnan(vy) || std::isnan(vz))
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "NaN encountered in ballistic fit.");
      return false;
    }
    return true;
  }

  bool TrajectoryPredictor::calculateImpactPoint(double lidar_to_target_x,
                                                 double lidar_to_target_y,
                                                 double lidar_to_target_z,
                                                 double z0, double vz,
                                                 double x0, double y0,
                                                 double vx, double vy,
                                                 double &impact_time,
                                                 double &x_impact,
                                                 double &y_impact)
  {
    double g = 9.81;
    double a = 0.5 * g;
    double b = -vz;
    double c = -z0 + lidar_to_target_z;
    double disc = b * b - 4 * a * c;
    if (disc < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "No real solution for impact time.");
      return false;
    }
    double sqrt_disc = std::sqrt(disc);
    double t1 = (-b + sqrt_disc) / (2 * a);
    double t2 = (-b - sqrt_disc) / (2 * a);
    if (t1 >= 0 && t2 >= 0)
      impact_time = std::min(t1, t2);
    else if (t1 >= 0)
      impact_time = t1;
    else if (t2 >= 0)
      impact_time = t2;
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "No positive impact time found.");
      return false;
    }
    x_impact = x0 + vx * impact_time + lidar_to_target_x;
    y_impact = y0 + vy * impact_time + lidar_to_target_y;
    return true;
  }

  std::vector<geometry_msgs::msg::Point> TrajectoryPredictor::generateTrajectoryPoints(double x0, double y0, double z0,
                                                                                       double vx, double vy, double vz,
                                                                                       double impact_time) const
  {
    std::vector<geometry_msgs::msg::Point> traj;
    double g = 9.81;
    double dt = 0.1;
    for (double t = 0.0; t <= impact_time; t += dt)
    {
      geometry_msgs::msg::Point pt;
      pt.x = x0 + vx * t;
      pt.y = y0 + vy * t;
      pt.z = z0 + vz * t - 0.5 * g * t * t;
      traj.push_back(pt);
    }
    // 最後の点の追加
    if (impact_time - (static_cast<int>(impact_time / dt) * dt) > 1e-6)
    {
      geometry_msgs::msg::Point pt;
      pt.x = x0 + vx * impact_time;
      pt.y = y0 + vy * impact_time;
      pt.z = z0 + vz * impact_time - 0.5 * g * impact_time * impact_time;
      traj.push_back(pt);
    }
    return traj;
  }

  double TrajectoryPredictor::evaluateCubic(const Eigen::VectorXd &coeffs, double t) const
  {
    return coeffs(0) * std::pow(t, 3) +
           coeffs(1) * std::pow(t, 2) +
           coeffs(2) * t +
           coeffs(3);
  }
} // namespace impact_point_estimator
