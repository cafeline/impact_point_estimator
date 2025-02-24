#include "impact_point_estimator/prediction.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <random>
#include <algorithm>

std::vector<geometry_msgs::msg::Point> Prediction::process_three_points(const std::vector<geometry_msgs::msg::Point> &points,
                                                                        double lidar_to_target_z,
                                                                        std::function<void(const PredictionResult &)> callback)
{
  Eigen::VectorXd coeffs_x, coeffs_y, coeffs_z;
  std::vector<geometry_msgs::msg::Point> curve_points = fit_cubic_curve(points, coeffs_x, coeffs_y, coeffs_z);
  if (curve_points.empty())
  {
    callback({false, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    return {};
  }

  double x_at_target, y_at_target;
  bool success = find_xy_at_target_height(coeffs_x, coeffs_y, coeffs_z, lidar_to_target_z, x_at_target, y_at_target);
  if (!success)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "目標高さでのx,y求解に失敗しました。");
    callback({false, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    return {};
  }

  RCLCPP_INFO(rclcpp::get_logger("Prediction"), "目標高さ z=%.3f における近似点: x=%.3f, y=%.3f", lidar_to_target_z, x_at_target, y_at_target);

  callback({true, 0.0, x_at_target, y_at_target, 0.0, 0.0, lidar_to_target_z, 0.0, 0.0, 0.0});
  return curve_points;
}

void Prediction::process_points(const std::vector<geometry_msgs::msg::Point> &points, double lidar_to_target_x, double lidar_to_target_y, double lidar_to_target_z, std::function<void(const PredictionResult &)> callback)
{
  // 物理モデルによる弾道軌道フィッティング
  double x0, y0, z0, vx, vy, vz;
  bool success = fit_ballistic_trajectory(points, timestamps_, x0, y0, z0, vx, vy, vz);
  if (!success)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "物理モデルによるフィッティングに失敗しました。");
    callback({false, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    return;
  }

  // RCLCPP_INFO(rclcpp::get_logger("Prediction"), "フィッティング結果: x0=%.6f, y0=%.6f, z0=%.6f, vx=%.6f, vy=%.6f, vz=%.6f", x0, y0, z0, vx, vy, vz);

  double impact_time, x_impact, y_impact;
  bool impact_success = calculate_impact_point(lidar_to_target_x, lidar_to_target_y, lidar_to_target_z, z0, vz, impact_time, x0, y0, vx, vy, x_impact, y_impact);
  if (!impact_success)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "有効な着弾時間または地点が見つかりませんでした。");
    callback({false, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    return;
  }

  callback({true, impact_time, x_impact, y_impact, x0, y0, z0, vx, vy, vz});
}

std::vector<geometry_msgs::msg::Point> Prediction::fit_cubic_curve(const std::vector<geometry_msgs::msg::Point> &points,
                                                                   Eigen::VectorXd &coeffs_x,
                                                                   Eigen::VectorXd &coeffs_y,
                                                                   Eigen::VectorXd &coeffs_z)
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
  double t_max = 5.0;        // 曲線を1.0倍先まで予測
  size_t total_points = 150; // より多くのポイントを生成

  for (size_t i = 0; i < total_points; ++i)
  {
    double t_val = static_cast<double>(i) / (total_points - 1) * t_max;
    double xi = evaluateCubic(coeffs_x, t_val);
    double yi = evaluateCubic(coeffs_y, t_val);
    double zi = evaluateCubic(coeffs_z, t_val);

    geometry_msgs::msg::Point pt;
    pt.x = xi;
    pt.y = yi;
    pt.z = zi;
    curve_points.push_back(pt);
  }

  return curve_points;
}

bool Prediction::find_xy_at_target_height(const Eigen::VectorXd &coeffs_x, const Eigen::VectorXd &coeffs_y, const Eigen::VectorXd &coeffs_z, double lidar_to_target_z, double &x_out, double &y_out)
{
  // t の探索範囲
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

  if (best_t < 0.0)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "目標高さに近似できる点が見つかりませんでした。");
    return false;
  }

  // 最も近いtでのx,yを計算
  x_out = evaluateCubic(coeffs_x, best_t);
  y_out = evaluateCubic(coeffs_y, best_t);

  return true;
}

double Prediction::calculate_time_to_height(const std::vector<geometry_msgs::msg::Point> &points, std::chrono::steady_clock::time_point start_time, std::chrono::steady_clock::time_point end_time, double lidar_to_target_z)
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
  // RCLCPP_INFO(rclcpp::get_logger("Prediction"), "初期位置 z0=%.2f m, 初期速度 v0=%.2f m/s, duration=%.2f s", z0, v0, duration.count());

  // 二次方程式の係数を設定
  // z = z0 + v0*t - (1/2)*g*t^2 から
  // (1/2)*g*t^2 - v0*t + (z0 - lidar_to_target_x) = 0 の形に変形
  double a = 0.5 * 9.81;             // 重力加速度の1/2
  double b = -v0;                    // 初期速度の負値
  double c = z0 - lidar_to_target_z; // 初期高さと目標高さの差

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

std::vector<geometry_msgs::msg::Point> Prediction::fit_cubic_curve_ransac(const std::vector<geometry_msgs::msg::Point> &points, Eigen::VectorXd &coeffs_x, Eigen::VectorXd &coeffs_y, Eigen::VectorXd &coeffs_z, double threshold, int max_iterations)
{
  size_t n = points.size();
  if (n < 4)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "RANSACには最低4点が必要です。フィッティングをスキップします。");
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
    // ランダムに4点を選択
    std::vector<int> indices;
    while (indices.size() < 4)
    {
      int idx = dis(gen);
      if (std::find(indices.begin(), indices.end(), idx) == indices.end())
      {
        indices.push_back(idx);
      }
    }

    // 選択した点で3次多項式をフィッティング
    std::vector<geometry_msgs::msg::Point> sample_points;
    for (const auto &idx : indices)
    {
      sample_points.push_back(points[idx]);
    }

    Eigen::VectorXd sample_coeffs_x, sample_coeffs_y, sample_coeffs_z;
    std::vector<geometry_msgs::msg::Point> fitted_points = fit_cubic_curve(sample_points, sample_coeffs_x, sample_coeffs_y, sample_coeffs_z);
    bool fit_success = !fitted_points.empty();
    if (!fit_success)
      continue;

    // インライアの数をカウント
    int inliers = 0;
    std::vector<int> inlier_indices;
    for (size_t i = 0; i < n; ++i)
    {
      // 各点のtを推定（ここでは点の順序を使用）
      double t = static_cast<double>(i) / (n - 1);

      // フィッティング曲線上の点を計算
      double fitted_x = evaluateCubic(sample_coeffs_x, t);
      double fitted_y = evaluateCubic(sample_coeffs_y, t);
      double fitted_z = evaluateCubic(sample_coeffs_z, t);

      // 実際の点との距離を計算
      double dist = std::hypot(points[i].x - fitted_x, std::hypot(points[i].y - fitted_y, points[i].z - fitted_z));

      if (dist < threshold)
      {
        inliers++;
        inlier_indices.push_back(i);
      }
    }

    // ベストモデルを更新
    if (inliers > best_inliers)
    {
      best_inliers = inliers;
      best_coeffs_x = sample_coeffs_x;
      best_coeffs_y = sample_coeffs_y;
      best_coeffs_z = sample_coeffs_z;
      best_inlier_indices = inlier_indices;

      // 早期終了条件
      if (best_inliers > 0.8 * n)
      {
        break;
      }
    }
  }

  // 最適なインライアを用いて再フィッティング
  if (best_inliers >= 4)
  {
    std::vector<geometry_msgs::msg::Point> inlier_points;
    for (const auto &idx : best_inlier_indices)
    {
      inlier_points.push_back(points[idx]);
    }

    std::vector<geometry_msgs::msg::Point> final_fitted_points = fit_cubic_curve(inlier_points, coeffs_x, coeffs_y, coeffs_z);
    bool final_fit = !final_fitted_points.empty();

    if (final_fit)
    {
      // フィッティングされた曲線の生成
      std::vector<geometry_msgs::msg::Point> curve_points;
      double t_max = 1.5;        // 曲線を1.5倍先まで予測
      size_t total_points = 150; // より多くのポイントを生成

      for (size_t i = 0; i < total_points; ++i)
      {
        double t_val = static_cast<double>(i) / (total_points - 1) * t_max;
        double xi = evaluateCubic(coeffs_x, t_val);
        double yi = evaluateCubic(coeffs_y, t_val);
        double zi = evaluateCubic(coeffs_z, t_val);

        geometry_msgs::msg::Point pt;
        pt.x = xi;
        pt.y = yi;
        pt.z = zi;
        curve_points.push_back(pt);
      }

      RCLCPP_INFO(rclcpp::get_logger("Prediction"), "RANSACによるフィッティングに成功しました。インライア数: %d", best_inliers);
      return curve_points;
    }
  }

  RCLCPP_WARN(rclcpp::get_logger("Prediction"), "RANSACによるフィッティングに失敗しました。");
  return {};
}

bool Prediction::fit_ballistic_trajectory(const std::vector<geometry_msgs::msg::Point> &points, const std::vector<double> &times, double &x0, double &y0, double &z0, double &vx, double &vy, double &vz)
{
  if (points.size() < 4)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "フィッティングに必要な点が不足しています");
    return false;
  }

  size_t N = points.size();
  double g = 9.81;

  // 以下のモデルでフィッティングする：
  // x(t)=x0+vx*t
  // y(t)=y0+vy*t
  // z(t)=z0+vz*t -0.5*g*t^2

  // 行列を構築
  // x方向: X = A_x * p_x, p_x=[x0, vx]^T
  // y方向: Y = A_y * p_y, p_y=[y0, vy]^T
  // z方向: Z = A_z * p_z, p_z=[z0, vz]^T
  // ただしz方向はZ[i]=z_i + 0.5*g*t_i^2にすれば線形
  // Z'[i] = z_i + 0.5*g*t_i^2 = z0 + vz*t_i

  Eigen::MatrixXd A_x(N, 2), A_y(N, 2), A_z(N, 2);
  Eigen::VectorXd X(N), Y(N), Z_mod(N);

  for (size_t i = 0; i < N; ++i)
  {
    double t = times[i];
    double xt = points[i].x;
    double yt = points[i].y;
    double zt = points[i].z;

    A_x(i, 0) = 1.0;
    A_x(i, 1) = t;
    A_y(i, 0) = 1.0;
    A_y(i, 1) = t;
    A_z(i, 0) = 1.0;
    A_z(i, 1) = t;

    X(i) = xt;
    Y(i) = yt;
    Z_mod(i) = zt + 0.5 * g * t * t;
  }

  // 正則方程式を解く
  Eigen::Vector2d p_x = A_x.colPivHouseholderQr().solve(X);
  Eigen::Vector2d p_y = A_y.colPivHouseholderQr().solve(Y);
  Eigen::Vector2d p_z = A_z.colPivHouseholderQr().solve(Z_mod);

  x0 = p_x(0);
  vx = p_x(1);
  y0 = p_y(0);
  vy = p_y(1);
  z0 = p_z(0);
  vz = p_z(1);

  // 基本的な妥当性チェック（例えばvzが極端すぎる場合など）
  if (std::isnan(x0) || std::isnan(y0) || std::isnan(z0) ||
      std::isnan(vx) || std::isnan(vy) || std::isnan(vz))
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "物理モデルフィット結果にNaNが含まれています。");
    return false;
  }

  return true;
}

bool Prediction::calculate_impact_point(double lidar_to_target_x, double lidar_to_target_y, double lidar_to_target_z, double z0, double vz, double &impact_time, double x0, double y0, double vx, double vy, double &x_impact, double &y_impact)
{
  // z(t) = z0 + vz * t - 0.5 * g * t^2
  // 着地時点では z(t) = lidar_to_target_z
  // => 0.5 * g * t^2 - vz * t + (z0 - lidar_to_target_z) = 0

  double a = 0.5 * 9.81;
  double b = -vz;
  double c = -z0 + lidar_to_target_z;

  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "着地時間の計算に失敗しました（判別式が負）。");
    return false;
  }

  double sqrt_discriminant = std::sqrt(discriminant);
  double t1 = (-b + sqrt_discriminant) / (2 * a);
  double t2 = (-b - sqrt_discriminant) / (2 * a);

  // 正の解を選択
  if (t1 >= 0 && t2 >= 0)
  {
    impact_time = std::min(t1, t2);
  }
  else if (t1 >= 0)
  {
    impact_time = t1;
  }
  else if (t2 >= 0)
  {
    impact_time = t2;
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "有効な着地時間が見つかりませんでした。");
    return false;
  }

  // 着地時点のx, y座標を計算
  x_impact = x0 + vx * impact_time + lidar_to_target_x;
  y_impact = y0 + vy * impact_time + lidar_to_target_y;

  return true;
}

double Prediction::calculate_relative_time(std::chrono::steady_clock::time_point current_time)
{
  if (!start_time_initialized_)
  {
    RCLCPP_WARN(rclcpp::get_logger("Prediction"), "開始時刻が初期化されていません。");
    return 0.0;
  }

  std::chrono::duration<double> elapsed = current_time - start_time_;
  return elapsed.count(); // 秒単位
}

void Prediction::set_start_time(std::chrono::steady_clock::time_point start_time)
{
  start_time_ = start_time;
  start_time_initialized_ = true;
}

bool Prediction::is_start_time_initialized() const
{
  return start_time_initialized_;
}

void Prediction::add_timestamp(double dt)
{
  timestamps_.emplace_back(dt);
}

std::vector<geometry_msgs::msg::Point> Prediction::generate_trajectory_points(double x0, double y0, double z0, double vx, double vy, double vz, double impact_time)
{
  std::vector<geometry_msgs::msg::Point> trajectory;
  double g = 9.81;        // 重力加速度 (m/s^2)
  double time_step = 0.1; // 秒単位の時間ステップ

  for (double t = 0.0; t <= impact_time; t += time_step)
  {
    geometry_msgs::msg::Point pt;
    pt.x = x0 + vx * t;
    pt.y = y0 + vy * t;
    pt.z = z0 + vz * t - 0.5 * g * t * t;
    trajectory.emplace_back(pt);
  }

  // 最後の点を確実に追加
  if (impact_time - (static_cast<int>(impact_time / time_step) * time_step) > 1e-6)
  {
    geometry_msgs::msg::Point final_pt;
    final_pt.x = x0 + vx * impact_time;
    final_pt.y = y0 + vy * impact_time;
    final_pt.z = z0 + vz * impact_time - 0.5 * g * impact_time * impact_time;
    trajectory.emplace_back(final_pt);
  }

  return trajectory;
}

double Prediction::evaluateCubic(const Eigen::VectorXd &coeffs, double t)
{
  return coeffs(0) * std::pow(t, 3) +
         coeffs(1) * std::pow(t, 2) +
         coeffs(2) * t +
         coeffs(3);
}