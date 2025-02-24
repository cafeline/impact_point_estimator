#include "impact_point_estimator/trajectory_predictor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <random>
#include <algorithm>

namespace impact_point_estimator
{

  std::vector<geometry_msgs::msg::Point> TrajectoryPredictor::fitCubicCurve(const std::vector<geometry_msgs::msg::Point> &points,
                                                                            Eigen::VectorXd &coeffs_x,
                                                                            Eigen::VectorXd &coeffs_y,
                                                                            Eigen::VectorXd &coeffs_z)
  {
    size_t n = points.size();
    if (n < 3)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "Not enough points for cubic fit.");
      return {};
    }
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
      x(i) = points[i].x;
      y(i) = points[i].y;
      z(i) = points[i].z;
    }
    coeffs_x = A.colPivHouseholderQr().solve(x);
    coeffs_y = A.colPivHouseholderQr().solve(y);
    coeffs_z = A.colPivHouseholderQr().solve(z);

    // フィッティングされた曲線の生成
    std::vector<geometry_msgs::msg::Point> curve;
    double t_max = 5.0;
    size_t total_points = 150;
    for (size_t i = 0; i < total_points; ++i)
    {
      double t_val = static_cast<double>(i) / (total_points - 1) * t_max;
      geometry_msgs::msg::Point pt;
      pt.x = evaluateCubic(coeffs_x, t_val);
      pt.y = evaluateCubic(coeffs_y, t_val);
      pt.z = evaluateCubic(coeffs_z, t_val);
      curve.push_back(pt);
    }
    return curve;
  }

  double TrajectoryPredictor::evaluateCubic(const Eigen::VectorXd &coeffs, double t) const
  {
    return coeffs(0) * std::pow(t, 3) +
           coeffs(1) * std::pow(t, 2) +
           coeffs(2) * t +
           coeffs(3);
  }

  std::vector<geometry_msgs::msg::Point> TrajectoryPredictor::fitCubicCurveRANSAC(const std::vector<geometry_msgs::msg::Point> &points,
                                                                                  Eigen::VectorXd &coeffs_x,
                                                                                  Eigen::VectorXd &coeffs_y,
                                                                                  Eigen::VectorXd &coeffs_z,
                                                                                  double threshold,
                                                                                  int max_iterations)
  {
    size_t n = points.size();
    if (n < 4)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "Not enough points for RANSAC cubic fit.");
      return {};
    }
    int best_inliers = 0;
    Eigen::VectorXd best_coeffs_x, best_coeffs_y, best_coeffs_z;
    std::vector<int> best_inlier_indices;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, n - 1);

    for (int iter = 0; iter < max_iterations; ++iter)
    {
      std::vector<int> indices;
      while (indices.size() < 4)
      {
        int idx = dis(gen);
        if (std::find(indices.begin(), indices.end(), idx) == indices.end())
        {
          indices.push_back(idx);
        }
      }
      std::vector<geometry_msgs::msg::Point> sample;
      for (int idx : indices)
      {
        sample.push_back(points[idx]);
      }
      Eigen::VectorXd sample_coeffs_x, sample_coeffs_y, sample_coeffs_z;
      auto fitted = fitCubicCurve(sample, sample_coeffs_x, sample_coeffs_y, sample_coeffs_z);
      if (fitted.empty())
        continue;
      int inliers = 0;
      std::vector<int> inlier_indices;
      for (size_t i = 0; i < n; ++i)
      {
        double t = static_cast<double>(i) / (n - 1);
        double fx = evaluateCubic(sample_coeffs_x, t);
        double fy = evaluateCubic(sample_coeffs_y, t);
        double fz = evaluateCubic(sample_coeffs_z, t);
        double dist = std::hypot(points[i].x - fx, std::hypot(points[i].y - fy, points[i].z - fz));
        if (dist < threshold)
        {
          inliers++;
          inlier_indices.push_back(i);
        }
      }
      if (inliers > best_inliers)
      {
        best_inliers = inliers;
        best_coeffs_x = sample_coeffs_x;
        best_coeffs_y = sample_coeffs_y;
        best_coeffs_z = sample_coeffs_z;
        best_inlier_indices = inlier_indices;
        if (best_inliers > 0.8 * n)
          break;
      }
    }
    if (best_inliers >= 4)
    {
      std::vector<geometry_msgs::msg::Point> inlier_points;
      for (int idx : best_inlier_indices)
      {
        inlier_points.push_back(points[idx]);
      }
      auto final_curve = fitCubicCurve(inlier_points, coeffs_x, coeffs_y, coeffs_z);
      if (!final_curve.empty())
      {
        std::vector<geometry_msgs::msg::Point> curve;
        double t_max = 1.5;
        size_t total_points = 150;
        for (size_t i = 0; i < total_points; ++i)
        {
          double t_val = static_cast<double>(i) / (total_points - 1) * t_max;
          geometry_msgs::msg::Point pt;
          pt.x = evaluateCubic(coeffs_x, t_val);
          pt.y = evaluateCubic(coeffs_y, t_val);
          pt.z = evaluateCubic(coeffs_z, t_val);
          curve.push_back(pt);
        }
        RCLCPP_INFO(rclcpp::get_logger("TrajectoryPredictor"), "RANSAC fit succeeded with %d inliers.", best_inliers);
        return curve;
      }
    }
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "RANSAC fit failed.");
    return {};
  }

  bool TrajectoryPredictor::fitBallisticTrajectory(const std::vector<geometry_msgs::msg::Point> &points,
                                                   const std::vector<double> &times,
                                                   double &x0, double &y0, double &z0,
                                                   double &vx, double &vy, double &vz)
  {
    if (points.size() < 4 || times.size() < points.size())
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
    double c = z0 - lidar_to_target_z;
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

  std::vector<geometry_msgs::msg::Point> TrajectoryPredictor::processThreePoints(const std::vector<geometry_msgs::msg::Point> &points,
                                                                                 double lidar_to_target_z,
                                                                                 PredictionCallback callback)
  {
    Eigen::VectorXd coeffs_x, coeffs_y, coeffs_z;
    auto curve = fitCubicCurve(points, coeffs_x, coeffs_y, coeffs_z);
    if (curve.empty())
    {
      callback({false, -1, 0, 0, 0, 0, 0, 0, 0, 0});
      return {};
    }
    // t の探索（目標高さに近いtを探索）
    double t_max = 1.5;
    size_t search_points = 1000;
    double best_diff = std::numeric_limits<double>::infinity();
    double best_t = -1.0;
    for (size_t i = 0; i < search_points; ++i)
    {
      double t_val = static_cast<double>(i) / (search_points - 1) * t_max;
      double z_val = evaluateCubic(coeffs_z, t_val);
      double diff = std::fabs(z_val - lidar_to_target_z);
      if (diff < best_diff)
      {
        best_diff = diff;
        best_t = t_val;
      }
    }
    if (best_t < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryPredictor"), "Failed to find suitable t for target height.");
      callback({false, -1, 0, 0, 0, 0, 0, 0, 0, 0});
      return {};
    }
    double x_at_target = evaluateCubic(coeffs_x, best_t);
    double y_at_target = evaluateCubic(coeffs_y, best_t);
    callback({true, 0, x_at_target, y_at_target, 0, 0, lidar_to_target_z, 0, 0, 0});
    return curve;
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

} // namespace impact_point_estimator
