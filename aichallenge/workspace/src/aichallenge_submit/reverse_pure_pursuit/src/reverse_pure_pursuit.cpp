#include "reverse_pure_pursuit/reverse_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <sstream>
#include <string>

namespace reverse_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

ReversePurePursuit::ReversePurePursuit()
: Node("reverse_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  lookahead_cte_gain_(declare_parameter<float>("lookahead_cte_gain", 3.0)),
  lookahead_curve_ref_(declare_parameter<float>("lookahead_curve_ref", 12.0)),
  lookahead_curve_min_(declare_parameter<float>("lookahead_curve_min", 0.5)),
  // 低速時の曲率による目標点縮小を効かせる速度の上限[m/s]
  lookahead_slow_speed_(declare_parameter<float>("lookahead_slow_speed", 2.78)),
  lookahead_slow_full_(declare_parameter<float>("lookahead_slow_full", 1.0)),
  // 縮めた目標距離 = 曲率半径 x この係数
  lookahead_curve_k_(declare_parameter<float>("lookahead_curve_k", 0.35)),
  lookahead_slow_min_(declare_parameter<float>("lookahead_slow_min", 1.5)),
  // 低速での縮め方の鋭さ。大きいほど停止に近い側で急激に短くなる。
  lookahead_slow_exp_(declare_parameter<float>("lookahead_slow_exp", 2.0)),
  // 低速でも直線ならここまでしか縮めない[m]
  lookahead_slow_far_(declare_parameter<float>("lookahead_slow_far", 3.0)),
  // 追い越し試行中に lookahead に掛ける倍率
  lookahead_overtake_scale_(declare_parameter<float>("lookahead_overtake_scale", 0.9)),
  lookahead_zone_spec_(declare_parameter<std::string>("lookahead_scale_zones", "")),
  lookahead_curve_ahead_(declare_parameter<float>("lookahead_curve_ahead", 6.0)),
  start_steer_speed_(declare_parameter<float>("start_steer_speed", 3.0)),
  start_steer_limit_(declare_parameter<float>("start_steer_limit", 0.21)),
  stuck_steer_free_speed_(declare_parameter<float>("stuck_steer_free_speed", 0.4)),
  max_acceleration_(declare_parameter<float>("max_acceleration", 3.0)),
  // 後退時の速度上限[m/s]
  reverse_max_speed_(declare_parameter<float>("reverse_max_speed", 1.5))
{
  // "165:185:0.35,10:20:0.5" の形を解析する
  {
    std::stringstream ss(lookahead_zone_spec_);
    std::string item;
    while (std::getline(ss, item, ',')) {
      std::size_t a = item.find(':');
      std::size_t b = item.rfind(':');
      if (a == std::string::npos || b == a) { continue; }
      LookaheadZone z;
      z.from = static_cast<std::size_t>(std::stoul(item.substr(0, a)));
      z.to = static_cast<std::size_t>(std::stoul(item.substr(a + 1, b - a - 1)));
      z.scale = std::stod(item.substr(b + 1));
      lookahead_zones_.push_back(z);
      RCLCPP_INFO(get_logger(), "lookahead 縮小区間 idx%zu-%zu x%.2f", z.from, z.to, z.scale);
    }
  }
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  // デバッグ用トピック名が前進用(simple_pure_pursuit)と衝突しないよう reverse_ を付ける
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/reverse_lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_overtaking_ = create_subscription<std_msgs::msg::Bool>(
    "input/overtaking", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { overtaking_ = msg->data; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ = create_wall_timer(10ms, std::bind(&ReversePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

// 自車の少し先までの軌道で最もきつい曲率半径[m]を3点の外接円で求める(直線は上限値)。
double ReversePurePursuit::localTurnRadius(size_t closest_idx) const
{
  constexpr double kStraight = 1e4;
  if (!trajectory_) {
    return kStraight;
  }
  const auto & pts = trajectory_->points;
  const size_t n = pts.size();
  if (n < 3) {
    return kStraight;
  }
  const auto at = [&](size_t i) { return pts.at(i % n).pose.position; };

  // 外接円を測る点の間隔[点](約2.7m相当)。
  size_t span = 1;
  {
    double total = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const auto & a = at(i);
      const auto & b = at(i + 1);
      total += std::hypot(b.x - a.x, b.y - a.y);
    }
    const double spacing = total / static_cast<double>(n);
    if (spacing > 1e-6) {
      span = std::max<size_t>(1, static_cast<size_t>(std::lround(2.7 / spacing)));
    }
  }

  // curve_ahead 先までで最もきつい曲率を採る。
  double tightest = kStraight;
  double travelled = 0.0;
  for (size_t k = 0; k + 2 < n; ++k) {
    const auto a = at(closest_idx + k);
    const auto b = at(closest_idx + k + span);
    const auto c = at(closest_idx + k + 2 * span);
    if (k > 0) {
      const auto prev = at(closest_idx + k - 1);
      travelled += std::hypot(a.x - prev.x, a.y - prev.y);
      if (travelled > lookahead_curve_ahead_) {
        break;
      }
    }
    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ca = std::hypot(a.x - c.x, a.y - c.y);
    // 三角形の面積(外積)。潰れていれば直線とみなす。
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (std::abs(cross) < 1e-6 || ab < 1e-6 || bc < 1e-6 || ca < 1e-6) {
      continue;
    }
    const double r = (ab * bc * ca) / (2.0 * std::abs(cross));
    tightest = std::min(tightest, r);
  }
  return tightest;
}

void ReversePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // get closest trajectory point from current position
  TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

  // calc longitudinal speed and acceleration
  // 経路速度を reverse_max_speed で頭打ちにし、符号を反転して後退指令にする。
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  target_longitudinal_vel = -std::min(std::abs(target_longitudinal_vel), reverse_max_speed_);
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  cmd.longitudinal.speed = target_longitudinal_vel;
  // 加速度はギアの向きへのペダル(正=後退方向へ加速、負=ブレーキ)として gain*(現在-目標) で出す。
  cmd.longitudinal.acceleration =
    speed_proportional_gain_ * (current_longitudinal_vel - target_longitudinal_vel);
  // 加減速とも ±max_acceleration に収める。
  cmd.longitudinal.acceleration =
    std::clamp<double>(cmd.longitudinal.acceleration, -max_acceleration_, max_acceleration_);

  // 横ずれが大きいときは lookahead を伸ばして緩やかに合流する。
  const double cross_track_error = std::hypot(
    closet_traj_point.pose.position.x - odometry_->pose.pose.position.x,
    closet_traj_point.pose.position.y - odometry_->pose.pose.position.y);
  // lookahead_scale_zones で指定した idx 区間では lookahead_min_distance を縮める。
  double curve_scale = 1.0;
  for (const auto & z : lookahead_zones_) {
    const bool inside = (z.from <= z.to)
                          ? (closet_traj_point_idx >= z.from && closet_traj_point_idx <= z.to)
                          : (closet_traj_point_idx >= z.from || closet_traj_point_idx <= z.to);
    if (inside) { curve_scale = std::min(curve_scale, z.scale); }
  }
  // 急に切り替えると舵が跳ねるので、なまして入れる
  lookahead_scale_now_ += (curve_scale - lookahead_scale_now_) * kScaleSmooth;
  curve_scale = lookahead_scale_now_;
  // 追い越し試行中は速度に依らない項だけを縮めて目標点を近づける。
  {
    const double want = overtaking_ ? lookahead_overtake_scale_ : 1.0;
    overtake_scale_now_ += (want - overtake_scale_now_) * kScaleSmooth;
  }
  double lookahead_distance = std::max(
    lookahead_gain_ * target_longitudinal_vel +
      lookahead_min_distance_ * curve_scale * overtake_scale_now_,
    lookahead_cte_gain_ * cross_track_error);

  // 低速時は速度と曲率に応じて lookahead に上限を掛ける。
  {
    const double v_now = std::abs(current_longitudinal_vel);
    double cap = 1e9;
    if (v_now < lookahead_slow_speed_) {
      const double radius = localTurnRadius(closet_traj_point_idx);
      // 基準速度以下では速度が落ちるほど距離を exp(-k*x) で縮める。
      const double x =
        std::clamp((lookahead_slow_speed_ - v_now) / lookahead_slow_speed_, 0.0, 1.0);
      const double k = std::max(lookahead_slow_exp_, 1e-3);
      const double by_speed = lookahead_distance * std::exp(-k * x);
      // コーナーでは曲率半径に比例した長さでも抑え、下限を切る。
      const double by_curve = radius * lookahead_curve_k_;
      cap = std::max(std::min(by_speed, by_curve), lookahead_slow_min_);
    }
    // 急に切り替えると舵が跳ねるので、なましてから掛ける。
    if (lookahead_slow_now_ > 1e8 && cap > 1e8) {
      lookahead_slow_now_ = cap;                    // どちらも無効。そのまま
    } else {
      const double target = (cap > 1e8) ? lookahead_distance * 4.0 : cap;
      if (lookahead_slow_now_ > 1e8) { lookahead_slow_now_ = target; }
      lookahead_slow_now_ += (target - lookahead_slow_now_) * kScaleSmooth;
    }
    if (lookahead_slow_now_ < 1e8) {
      lookahead_distance = std::min(lookahead_distance, lookahead_slow_now_);
    }
    lookahead_distance = std::max(lookahead_distance, lookahead_slow_min_);
  }
  // 前軸位置を基準点にする(変数名は rear_x/rear_y のまま)。
  const double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  double rear_x = odometry_->pose.pose.position.x + wheel_base_ / 2.0 * std::cos(yaw);
  double rear_y = odometry_->pose.pose.position.y + wheel_base_ / 2.0 * std::sin(yaw);
  // 最近傍点からインデックス増加方向へ lookahead 点を探す(閉ループは先頭へ回り込む)。
  const auto & traj_points = trajectory_->points;
  const size_t n_points = traj_points.size();
  // 先頭と末尾の隙間が平均点間隔の2倍以内なら閉ループとみなす。
  bool is_closed_loop = false;
  if (n_points > 3) {
    double span = 0.0;
    for (size_t i = 0; i + 1 < n_points; ++i) {
      span += std::hypot(
        traj_points[i + 1].pose.position.x - traj_points[i].pose.position.x,
        traj_points[i + 1].pose.position.y - traj_points[i].pose.position.y);
    }
    const double mean_spacing = span / static_cast<double>(n_points - 1);
    const double end_gap = std::hypot(
      traj_points.front().pose.position.x - traj_points.back().pose.position.x,
      traj_points.front().pose.position.y - traj_points.back().pose.position.y);
    is_closed_loop = end_gap < mean_spacing * 2.0;
  }

  size_t lookahead_idx = closet_traj_point_idx;
  bool lookahead_found = false;
  const size_t search_count = is_closed_loop ? n_points : (n_points - closet_traj_point_idx);
  for (size_t k = 0; k < search_count; ++k) {
    const size_t i =
      is_closed_loop ? (closet_traj_point_idx + k) % n_points : (closet_traj_point_idx + k);
    const auto & p = traj_points.at(i).pose.position;
    if (std::hypot(p.x - rear_x, p.y - rear_y) >= lookahead_distance) {
      lookahead_idx = i;
      lookahead_found = true;
      break;
    }
  }
  // 全点が lookahead 以内なら終端を使う
  if (!lookahead_found) {
    lookahead_idx = n_points - 1;
  }
  double lookahead_point_x = traj_points.at(lookahead_idx).pose.position.x;
  double lookahead_point_y = traj_points.at(lookahead_idx).pose.position.y;

  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = lookahead_point_x;
  lookahead_point_msg.point.y = lookahead_point_y;
  lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // calc steering angle for lateral control
  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                 yaw;
  cmd.lateral.steering_tire_angle =
    steering_tire_angle_gain_ * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
  // alpha は車体前方基準なので後退でも舵の符号反転は不要。

  // 低速時は操舵角に上限を掛ける。
  {
    const double v_abs = std::abs(current_longitudinal_vel);
    // ほぼ停止中は脱出のため制限を外す。
    if (v_abs > stuck_steer_free_speed_ && v_abs < start_steer_speed_) {
      // 停止時 start_steer_limit_ から、しきい速度で通常制限まで線形に開放する
      const double ratio = (start_steer_speed_ > 1e-6) ? (v_abs / start_steer_speed_) : 1.0;
      const double limit = start_steer_limit_ + ratio * (M_PI_2 - start_steer_limit_);
      cmd.lateral.steering_tire_angle =
        std::clamp<double>(cmd.lateral.steering_tire_angle, -limit, limit);
    }
  }

  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

bool ReversePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}
}  // namespace reverse_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<reverse_pure_pursuit::ReversePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
