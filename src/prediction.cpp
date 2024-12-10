#include "impact_point_estimator/prediction.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace impact_point_estimator
{
  std::vector<geometry_msgs::msg::Point> Prediction::fit_cubic_curve(const std::vector<geometry_msgs::msg::Point> &points, Eigen::VectorXd &coeffs_x, Eigen::VectorXd &coeffs_y, Eigen::VectorXd &coeffs_z)
  {
    size_t n = points.size();
    if (n < 3)
    {
      RCLCPP_WARN(rclcpp::get_logger("Prediction"), "十分な点がありません。カーブのフィッティングをスキップします。");
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

    // フィットされた曲線の生成
    std::vector<geometry_msgs::msg::Point> curve_points;
    for (size_t i = 0; i < 100; ++i)
    {
      double ti = static_cast<double>(i) / 99.0;
      double xi = coeffs_x(0) * std::pow(ti, 3) + coeffs_x(1) * std::pow(ti, 2) + coeffs_x(2) * ti + coeffs_x(3);
      double yi = coeffs_y(0) * std::pow(ti, 3) + coeffs_y(1) * std::pow(ti, 2) + coeffs_y(2) * ti + coeffs_y(3);
      double zi = coeffs_z(0) * std::pow(ti, 3) + coeffs_z(1) * std::pow(ti, 2) + coeffs_z(2) * ti + coeffs_z(3);

      geometry_msgs::msg::Point pt;
      pt.x = xi;
      pt.y = yi;
      pt.z = zi;
      curve_points.push_back(pt);
    }

    return curve_points;
  }

  double Prediction::calculate_time_to_height(const std::vector<geometry_msgs::msg::Point> &points, std::chrono::steady_clock::time_point start_time, std::chrono::steady_clock::time_point end_time, double target_height)
  {
    // 点が2つ未満の場合は計算不可能
    if (points.size() < 2)
    {
      RCLCPP_WARN(rclcpp::get_logger("Prediction"), "十分な点がありません。着弾時間を計算できません。");
      return -1.0;
    }

    // 計算に使用する最初と最後の点を取得
    auto start_point = points.front();
    auto end_point = points.back();

    // 最初の点から最後の点までの経過時間を計算
    std::chrono::duration<double> duration = end_time - start_time;
    double total_time = duration.count(); // 秒単位に変換

    // 初期高さを最初の点のz座標から設定
    double z0 = start_point.z;

    // 初期速度を計算
    // 公式: v0 = (z - z0 + 1/2*g*t^2) / t を使用
    // ここでzは最後の点の高さ、tは経過時間
    double v0 = (end_point.z - z0 + 0.5 * 9.81 * total_time * total_time) / total_time;

    // 計算に使用した初期値をログ出力
    RCLCPP_INFO(rclcpp::get_logger("Prediction"), "初期位置 z0=%.2f m, 初期速度 v0=%.2f m/s, duration=%.2f s", z0, v0, duration.count());

    // 二次方程式の係数を設定
    // z = z0 + v0*t - (1/2)*g*t^2 から
    // (1/2)*g*t^2 - v0*t + (z0 - z_target) = 0 の形に変形
    double a = 0.5 * 9.81;         // 重力加速度の1/2
    double b = -v0;                // 初期速度の負値
    double c = z0 - target_height; // 初期高さと目標高さの差

    // 判別式の計算 (b^2 - 4ac)
    double discriminant = b * b - 4 * a * c;

    // 判別式が負の場合、解なし（目標高さに到達しない）
    if (discriminant < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("Prediction"), "目標高さに達しません。");
      return -1.0;
    }

    // 二次方程式の解を計算
    double sqrt_discriminant = std::sqrt(discriminant);
    double t1 = (-b + sqrt_discriminant) / (2 * a); // 解1
    double t2 = (-b - sqrt_discriminant) / (2 * a); // 解2

    // 有効な解（着弾時間）を初期化
    double impact_time = -1.0;

    // 正の解のうち、小さい方を選択
    if (t1 >= 0 && t2 >= 0)
    {
      impact_time = std::min(t1, t2);
    }
    else if (t1 >= 0) // t1のみが正の場合
    {
      impact_time = t1;
    }
    else if (t2 >= 0) // t2のみが正の場合
    {
      impact_time = t2;
    }

    // 有効な解が見つかった場合
    if (impact_time >= 0)
    {
      return impact_time;
    }
    else // 有効な解が見つからなかった場合
    {
      RCLCPP_WARN(rclcpp::get_logger("Prediction"), "有効な着弾時間を見つけられませんでした。");
      return -1.0;
    }
  }

  std::tuple<double, double, double> Prediction::calculate_centroid(const std::vector<geometry_msgs::msg::Point> &points)
  {
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto &pt : points)
    {
      sum_x += pt.x;
      sum_y += pt.y;
      sum_z += pt.z;
    }
    size_t n = points.size();
    return {sum_x / n, sum_y / n, sum_z / n};
  }
}