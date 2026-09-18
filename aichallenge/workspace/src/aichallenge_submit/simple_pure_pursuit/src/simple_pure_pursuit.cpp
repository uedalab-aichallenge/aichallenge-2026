#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <sstream>
#include <string>
#include <utility>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

namespace
{

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool parseLookaheadZones(
  const std::string & spec,
  std::vector<SimplePurePursuit::LookaheadZone> & zones,
  std::string & reason)
{
  zones.clear();
  if (spec.empty()) {
    return true;
  }

  std::stringstream ss(spec);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    const auto first_colon = item.find(':');
    const auto second_colon = item.find(':', first_colon == std::string::npos ? 0 : first_colon + 1);
    if (item.empty() || first_colon == std::string::npos || second_colon == std::string::npos ||
        item.find(':', second_colon + 1) != std::string::npos) {
      reason = "expected comma-separated from:to:scale entries";
      return false;
    }

    const auto from_text = trim(item.substr(0, first_colon));
    const auto to_text = trim(item.substr(first_colon + 1, second_colon - first_colon - 1));
    const auto scale_text = trim(item.substr(second_colon + 1));
    if (from_text.empty() || to_text.empty() || scale_text.empty() ||
        !std::all_of(from_text.begin(), from_text.end(), [](const char c) {
          return std::isdigit(static_cast<unsigned char>(c)) != 0;
        }) ||
        !std::all_of(to_text.begin(), to_text.end(), [](const char c) {
          return std::isdigit(static_cast<unsigned char>(c)) != 0;
        })) {
      reason = "zone indices must be unsigned integers";
      return false;
    }

    try {
      std::size_t from_consumed = 0;
      std::size_t to_consumed = 0;
      std::size_t from = static_cast<std::size_t>(std::stoull(from_text, &from_consumed));
      std::size_t to = static_cast<std::size_t>(std::stoull(to_text, &to_consumed));
      const double scale = std::stod(scale_text);
      if (from_consumed != from_text.size() || to_consumed != to_text.size() ||
          !std::isfinite(scale) || from > to || scale <= 0.0 || scale > 1000.0) {
        reason = "zone range or scale is invalid";
        return false;
      }
      zones.push_back({from, to, scale});
    } catch (const std::exception &) {
      reason = "zone index or scale is out of range";
      return false;
    }
  }
  return true;
}

bool readNumericParameter(const rclcpp::Parameter & parameter, double & value)
{
  try {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      value = parameter.as_double();
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      value = static_cast<double>(parameter.as_int());
    } else {
      return false;
    }
  } catch (const std::exception &) {
    return false;
  }
  return std::isfinite(value);
}

bool readBoolParameter(const rclcpp::Parameter & parameter, bool & value)
{
  try {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      return false;
    }
    value = parameter.as_bool();
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult rejectParameter(const std::string & reason)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = reason;
  return result;
}

}  // namespace

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  lookahead_cte_gain_(declare_parameter<float>("lookahead_cte_gain", 3.0)),
  // この速度[m/s]未満のコーナーで目標点を近づける。
  lookahead_slow_speed_(declare_parameter<float>("lookahead_slow_speed", 2.78)),
  // 低速時の目標距離の上限 = 曲率半径 x この係数。
  lookahead_curve_k_(declare_parameter<float>("lookahead_curve_k", 0.35)),
  lookahead_curve_always_(declare_parameter<bool>("lookahead_curve_always", false)),
  lookahead_curve_min_(declare_parameter<double>("lookahead_curve_min", 2.8)),
  lookahead_curve_max_(declare_parameter<double>("lookahead_curve_max", 6.0)),
  lookahead_slow_min_(declare_parameter<float>("lookahead_slow_min", 1.5)),
  // 低速時に目標距離を縮める指数。
  lookahead_slow_exp_(declare_parameter<float>("lookahead_slow_exp", 2.0)),
  // 追い越し中に lookahead の定数項へ掛ける倍率。
  lookahead_overtake_scale_(declare_parameter<float>("lookahead_overtake_scale", 0.9)),
  lookahead_zone_spec_(declare_parameter<std::string>("lookahead_scale_zones", "")),
  lookahead_curve_ahead_(declare_parameter<float>("lookahead_curve_ahead", 6.0)),
  start_steer_speed_(declare_parameter<float>("start_steer_speed", 3.0)),
  start_steer_limit_(declare_parameter<float>("start_steer_limit", 0.21)),
  stuck_steer_free_speed_(declare_parameter<float>("stuck_steer_free_speed", 0.4)),
  // 要求舵が実舵上限を超える間は前へ加速しない。
  sat_accel_guard_(declare_parameter<bool>("sat_accel_guard", true)),
  sat_steer_rad_(declare_parameter<double>("sat_steer_rad", 0.31)),
  sat_accel_max_(declare_parameter<double>("sat_accel_max", 0.0)),
  sat_guard_min_speed_(declare_parameter<double>("sat_guard_min_speed", 2.0)),
  max_acceleration_(declare_parameter<float>("max_acceleration", 3.0)),
  wall_guard_clamp_enable_(declare_parameter<bool>("wall_guard_clamp_enable", true)),
  wall_guard_stale_sec_(declare_parameter<float>("wall_guard_stale_sec", 0.3))
{
  // 区間指定「開始:終了:倍率」を解析する。
  {
    std::string zone_reason;
    if (!parseLookaheadZones(lookahead_zone_spec_, lookahead_zones_, zone_reason)) {
      RCLCPP_ERROR(get_logger(), "invalid lookahead_scale_zones: %s", zone_reason.c_str());
      lookahead_zone_spec_.clear();
    }
    for (const auto & zone : lookahead_zones_) {
      RCLCPP_INFO(
        get_logger(), "lookahead 縮小区間 idx%zu-%zu x%.2f", zone.from, zone.to, zone.scale);
    }
  }
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);
  // 壁ガードの舵角範囲を受け取り、上書きの有無を返す。
  pub_steer_override_ = create_publisher<std_msgs::msg::Bool>(
    "/control/wall_guard/override", rclcpp::QoS(1));
  sub_steer_limit_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/control/wall_guard/steer_limit", rclcpp::QoS(1),
    [this](const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
      steer_limit_lo_ = msg->vector.x;
      steer_limit_hi_ = msg->vector.y;
      steer_limit_flag_ = msg->vector.z;
      steer_limit_time_ = this->now();
      steer_limit_valid_ = true;
    });

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_overtaking_ = create_subscription<std_msgs::msg::Bool>(
    "input/overtaking", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { overtaking_ = msg->data; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ = create_wall_timer(10ms, std::bind(&SimplePurePursuit::onTimer, this));
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&SimplePurePursuit::onParameterSet, this, std::placeholders::_1));
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

// 自車の先 lookahead_curve_ahead_ [m] までで最もきつい曲率半径[m]を返す(直線は上限値)。
double SimplePurePursuit::localTurnRadius(size_t closest_idx) const
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

  // 外接円を測る点の間隔[点](約2.7m)。
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
    // 3点がほぼ一直線なら飛ばす。
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (std::abs(cross) < 1e-6 || ab < 1e-6 || bc < 1e-6 || ca < 1e-6) {
      continue;
    }
    const double r = (ab * bc * ca) / (2.0 * std::abs(cross));
    tightest = std::min(tightest, r);
  }
  return tightest;
}

void SimplePurePursuit::onTimer()
{
  std::lock_guard<std::mutex> parameter_lock(parameters_mutex_);
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
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  cmd.longitudinal.speed = target_longitudinal_vel;
  cmd.longitudinal.acceleration =
    speed_proportional_gain_ * (target_longitudinal_vel - current_longitudinal_vel);
  // 加減速を同じ上限に収める。
  cmd.longitudinal.acceleration =
    std::clamp<double>(cmd.longitudinal.acceleration, -max_acceleration_, max_acceleration_);

  // calc lateral control
  // 横ずれが大きいときは lookahead を伸ばす。
  const double cross_track_error = std::hypot(
    closet_traj_point.pose.position.x - odometry_->pose.pose.position.x,
    closet_traj_point.pose.position.y - odometry_->pose.pose.position.y);
  // 指定区間では lookahead の定数項を縮める。
  double curve_scale = 1.0;
  for (const auto & z : lookahead_zones_) {
    const bool inside = (z.from <= z.to)
                          ? (closet_traj_point_idx >= z.from && closet_traj_point_idx <= z.to)
                          : (closet_traj_point_idx >= z.from || closet_traj_point_idx <= z.to);
    if (inside) { curve_scale = std::min(curve_scale, z.scale); }
  }
  lookahead_scale_now_ += (curve_scale - lookahead_scale_now_) * kScaleSmooth;
  curve_scale = lookahead_scale_now_;
  // 追い越し中の倍率は定数項だけに掛ける。
  {
    const double want = overtaking_ ? lookahead_overtake_scale_ : 1.0;
    overtake_scale_now_ += (want - overtake_scale_now_) * kScaleSmooth;
  }
  double lookahead_distance = std::max(
    lookahead_gain_ * target_longitudinal_vel +
      lookahead_min_distance_ * curve_scale * overtake_scale_now_,
    lookahead_cte_gain_ * cross_track_error);

  // 低速時は速度と曲率半径から lookahead に上限を掛け、なまして適用する。
  {
    const double v_now = std::abs(current_longitudinal_vel);
    double cap = 1e9;
    // 曲率の上限は速度に関わらず掛ける(下限・上限の範囲で)。
    if (lookahead_curve_always_) {
      const double radius_a = localTurnRadius(closet_traj_point_idx);
      const double by_curve_a = std::clamp(
        radius_a * static_cast<double>(lookahead_curve_k_),
        lookahead_curve_min_, lookahead_curve_max_);
      cap = std::min(cap, by_curve_a);
    }
    if (v_now < lookahead_slow_speed_) {
      const double radius = localTurnRadius(closet_traj_point_idx);
      const double x =
        std::clamp((lookahead_slow_speed_ - v_now) / lookahead_slow_speed_, 0.0, 1.0);
      const double k = std::max(lookahead_slow_exp_, 1e-3);
      const double by_speed = lookahead_distance * std::exp(-k * x);
      const double by_curve = radius * lookahead_curve_k_;
      cap = std::max(std::min({by_speed, by_curve, cap}), lookahead_slow_min_);
    }
    if (lookahead_slow_now_ > 1e8 && cap > 1e8) {
      lookahead_slow_now_ = cap;
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
  // 後輪中心を yaw から求める。
  const double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  double rear_x = odometry_->pose.pose.position.x - wheel_base_ / 2.0 * std::cos(yaw);
  double rear_y = odometry_->pose.pose.position.y - wheel_base_ / 2.0 * std::sin(yaw);
  const auto & traj_points = trajectory_->points;
  const size_t n_points = traj_points.size();
  // 先頭と末尾の隙間が平均点間隔の2倍未満なら閉ループとみなし、末尾から先頭へ回り込んで探す。
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
  // 全点が lookahead 以内なら終端を使う。
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
  const double requested_steer = cmd.lateral.steering_tire_angle;

  // 低速時は操舵角に上限を掛け、速度に比例して開放する。
  {
    const double v_abs = std::abs(current_longitudinal_vel);
    // ほぼ停止しているときは制限しない。
    if (v_abs > stuck_steer_free_speed_ && v_abs < start_steer_speed_) {
      const double ratio = (start_steer_speed_ > 1e-6) ? (v_abs / start_steer_speed_) : 1.0;
      const double limit = start_steer_limit_ + ratio * (M_PI_2 - start_steer_limit_);
      cmd.lateral.steering_tire_angle =
        std::clamp<double>(cmd.lateral.steering_tire_angle, -limit, limit);
    }
  }

  // 壁ガードの舵角範囲でクランプする。無効・未受信・古い・範囲不正のときはクランプしない。
  {
    bool override_active = false;
    if (wall_guard_clamp_enable_ && steer_limit_valid_) {
      const double age = (this->now() - steer_limit_time_).seconds();
      const double lo = steer_limit_lo_;
      const double hi = steer_limit_hi_;
      if (age >= 0.0 && age < wall_guard_stale_sec_ &&
          std::isfinite(lo) && std::isfinite(hi) && lo <= hi) {
        const double before = cmd.lateral.steering_tire_angle;
        const double after = std::clamp<double>(before, lo, hi);
        if (after != before) {
          cmd.lateral.steering_tire_angle = after;
          override_active = true;
          const auto now_log = this->now();
          if ((now_log - last_steer_override_log_).seconds() > 2.0) {
            last_steer_override_log_ = now_log;
            RCLCPP_WARN(get_logger(),
              "舵角上書き 元=%.3frad(%.1fdeg) -> %.3frad(%.1fdeg) 範囲=[%.3f,%.3f] 検知=%.0f",
              before, before * 180.0 / M_PI, after, after * 180.0 / M_PI,
              lo, hi, steer_limit_flag_);
          }
        }
      }
    }
    std_msgs::msg::Bool ov;
    ov.data = override_active;
    pub_steer_override_->publish(ov);
  }

  // 大舵または急反転時に、目標点と最終舵を記録する。
  const bool large_steer = std::abs(requested_steer) > 25.0 * M_PI / 180.0;
  const bool sharp_reversal = requested_steer * prev_requested_steer_ < 0.0 &&
    std::abs(requested_steer - prev_requested_steer_) > 20.0 * M_PI / 180.0;
  if ((large_steer || sharp_reversal) &&
      (this->now() - last_steer_diag_log_).seconds() > 0.25)
  {
    last_steer_diag_log_ = this->now();
    RCLCPP_WARN(get_logger(),
      "操舵診断 nearest=%zu lookahead=%zu 距離=%.2fm alpha=%+.1fdeg "
      "要求=%+.1fdeg 前回=%+.1fdeg 最終=%+.1fdeg 速度=%.1fkm/h "
      "追越=%d 曲率倍率=%.2f 目標=(%.2f,%.2f)",
      closet_traj_point_idx, lookahead_idx, lookahead_distance,
      alpha * 180.0 / M_PI, requested_steer * 180.0 / M_PI,
      prev_requested_steer_ * 180.0 / M_PI,
      static_cast<double>(cmd.lateral.steering_tire_angle) * 180.0 / M_PI,
      current_longitudinal_vel * 3.6, overtaking_ ? 1 : 0,
      lookahead_scale_now_, lookahead_point_x, lookahead_point_y);
  }
  prev_requested_steer_ = requested_steer;

  // 要求舵が実舵上限を超えている間は前向きの加速度を抑える。
  if (sat_accel_guard_) {
    const double v_abs_for_guard = std::abs(current_longitudinal_vel);
    const double phys =
      std::abs(static_cast<double>(cmd.lateral.steering_tire_angle)) /
      std::max(static_cast<double>(steering_tire_angle_gain_), 1e-6);
    // sat_guard_min_speed_ 未満では効かせない。
    if (v_abs_for_guard > sat_guard_min_speed_ &&
        phys > sat_steer_rad_ && cmd.longitudinal.acceleration > sat_accel_max_) {
      const double before = cmd.longitudinal.acceleration;
      cmd.longitudinal.acceleration = static_cast<float>(sat_accel_max_);
      const auto tnow = this->now();
      if ((tnow - last_sat_log_).seconds() > 1.0) {
        last_sat_log_ = tnow;
        RCLCPP_WARN(get_logger(),
          "舵が飽和しているので加速を抑える 要求実舵%.1fdeg(上限%.1f) "
          "加速度 %.2f -> %.2f 自車%.1fkm/h",
          phys * 180.0 / M_PI, sat_steer_rad_ * 180.0 / M_PI,
          before, cmd.longitudinal.acceleration, current_longitudinal_vel * 3.6);
      }
    }
  }

  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

rcl_interfaces::msg::SetParametersResult SimplePurePursuit::onParameterSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  try {
    std::lock_guard<std::mutex> parameter_lock(parameters_mutex_);

    // Stage every value first.  Nothing is written to the live controller until all
    // requested values and the zone specification have passed validation.
    double wheel_base = wheel_base_;
    double lookahead_gain = lookahead_gain_;
    double lookahead_min_distance = lookahead_min_distance_;
    double speed_proportional_gain = speed_proportional_gain_;
    bool use_external_target_vel = use_external_target_vel_;
    double external_target_vel = external_target_vel_;
    double steering_tire_angle_gain = steering_tire_angle_gain_;
    double lookahead_cte_gain = lookahead_cte_gain_;
    double lookahead_slow_speed = lookahead_slow_speed_;
    double lookahead_curve_k = lookahead_curve_k_;
    double lookahead_slow_min = lookahead_slow_min_;
    double lookahead_slow_exp = lookahead_slow_exp_;
    double lookahead_overtake_scale = lookahead_overtake_scale_;
    double lookahead_curve_ahead = lookahead_curve_ahead_;
    double start_steer_speed = start_steer_speed_;
    double start_steer_limit = start_steer_limit_;
    double stuck_steer_free_speed = stuck_steer_free_speed_;
    bool sat_accel_guard = sat_accel_guard_;
    double sat_steer_rad = sat_steer_rad_;
    double sat_accel_max = sat_accel_max_;
    double sat_guard_min_speed = sat_guard_min_speed_;
    double max_acceleration = max_acceleration_;
    bool wall_guard_clamp_enable = wall_guard_clamp_enable_;
    double wall_guard_stale_sec = wall_guard_stale_sec_;
    std::string lookahead_zone_spec = lookahead_zone_spec_;

    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      double number = 0.0;
      if (name == "wheel_base") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("wheel_base must be numeric");
        }
        wheel_base = number;
      } else if (name == "lookahead_gain") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_gain must be numeric");
        }
        lookahead_gain = number;
      } else if (name == "lookahead_min_distance") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_min_distance must be numeric");
        }
        lookahead_min_distance = number;
      } else if (name == "speed_proportional_gain") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("speed_proportional_gain must be numeric");
        }
        speed_proportional_gain = number;
      } else if (name == "use_external_target_vel") {
        if (!readBoolParameter(parameter, use_external_target_vel)) {
          return rejectParameter("use_external_target_vel must be bool");
        }
      } else if (name == "external_target_vel") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("external_target_vel must be numeric");
        }
        external_target_vel = number;
      } else if (name == "steering_tire_angle_gain") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("steering_tire_angle_gain must be numeric");
        }
        steering_tire_angle_gain = number;
      } else if (name == "lookahead_cte_gain") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_cte_gain must be numeric");
        }
        lookahead_cte_gain = number;
      } else if (name == "lookahead_slow_speed") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_slow_speed must be numeric");
        }
        lookahead_slow_speed = number;
      } else if (name == "lookahead_curve_k") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_curve_k must be numeric");
        }
        lookahead_curve_k = number;
      } else if (name == "lookahead_slow_min") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_slow_min must be numeric");
        }
        lookahead_slow_min = number;
      } else if (name == "lookahead_slow_exp") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_slow_exp must be numeric");
        }
        lookahead_slow_exp = number;
      } else if (name == "lookahead_overtake_scale") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_overtake_scale must be numeric");
        }
        lookahead_overtake_scale = number;
      } else if (name == "lookahead_scale_zones") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          return rejectParameter("lookahead_scale_zones must be a string");
        }
        lookahead_zone_spec = parameter.as_string();
      } else if (name == "lookahead_curve_ahead") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("lookahead_curve_ahead must be numeric");
        }
        lookahead_curve_ahead = number;
      } else if (name == "start_steer_speed") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("start_steer_speed must be numeric");
        }
        start_steer_speed = number;
      } else if (name == "start_steer_limit") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("start_steer_limit must be numeric");
        }
        start_steer_limit = number;
      } else if (name == "stuck_steer_free_speed") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("stuck_steer_free_speed must be numeric");
        }
        stuck_steer_free_speed = number;
      } else if (name == "sat_accel_guard") {
        if (!readBoolParameter(parameter, sat_accel_guard)) {
          return rejectParameter("sat_accel_guard must be bool");
        }
      } else if (name == "sat_steer_rad") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("sat_steer_rad must be numeric");
        }
        sat_steer_rad = number;
      } else if (name == "sat_accel_max") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("sat_accel_max must be numeric");
        }
        sat_accel_max = number;
      } else if (name == "sat_guard_min_speed") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("sat_guard_min_speed must be numeric");
        }
        sat_guard_min_speed = number;
      } else if (name == "max_acceleration") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("max_acceleration must be numeric");
        }
        max_acceleration = number;
      } else if (name == "wall_guard_clamp_enable") {
        if (!readBoolParameter(parameter, wall_guard_clamp_enable)) {
          return rejectParameter("wall_guard_clamp_enable must be bool");
        }
      } else if (name == "wall_guard_stale_sec") {
        if (!readNumericParameter(parameter, number)) {
          return rejectParameter("wall_guard_stale_sec must be numeric");
        }
        wall_guard_stale_sec = number;
      }
      // Parameters owned by another callback are intentionally ignored.
    }

    if (!(wheel_base > 0.0)) { return rejectParameter("wheel_base must be > 0"); }
    if (lookahead_gain < 0.0) { return rejectParameter("lookahead_gain must be >= 0"); }
    if (!(lookahead_min_distance > 0.0)) { return rejectParameter("lookahead_min_distance must be > 0"); }
    if (speed_proportional_gain < 0.0) { return rejectParameter("speed_proportional_gain must be >= 0"); }
    if (external_target_vel < 0.0) { return rejectParameter("external_target_vel must be >= 0"); }
    if (!(steering_tire_angle_gain > 0.0)) { return rejectParameter("steering_tire_angle_gain must be > 0"); }
    if (lookahead_cte_gain < 0.0) { return rejectParameter("lookahead_cte_gain must be >= 0"); }
    if (!(lookahead_slow_speed > 0.0)) { return rejectParameter("lookahead_slow_speed must be > 0"); }
    if (!(lookahead_curve_k > 0.0)) { return rejectParameter("lookahead_curve_k must be > 0"); }
    if (!(lookahead_slow_min > 0.0)) { return rejectParameter("lookahead_slow_min must be > 0"); }
    if (!(lookahead_slow_exp > 0.0)) { return rejectParameter("lookahead_slow_exp must be > 0"); }
    if (!(lookahead_overtake_scale > 0.0)) { return rejectParameter("lookahead_overtake_scale must be > 0"); }
    if (!(lookahead_curve_ahead > 0.0)) { return rejectParameter("lookahead_curve_ahead must be > 0"); }
    if (!(start_steer_speed > 0.0)) { return rejectParameter("start_steer_speed must be > 0"); }
    if (!(start_steer_limit > 0.0) || start_steer_limit > M_PI_2) {
      return rejectParameter("start_steer_limit must be in (0, pi/2]");
    }
    if (stuck_steer_free_speed < 0.0) { return rejectParameter("stuck_steer_free_speed must be >= 0"); }
    if (!(sat_steer_rad > 0.0)) { return rejectParameter("sat_steer_rad must be > 0"); }
    if (sat_accel_max < 0.0) { return rejectParameter("sat_accel_max must be >= 0"); }
    if (sat_guard_min_speed < 0.0) { return rejectParameter("sat_guard_min_speed must be >= 0"); }
    if (!(max_acceleration > 0.0)) { return rejectParameter("max_acceleration must be > 0"); }
    if (!(wall_guard_stale_sec > 0.0)) { return rejectParameter("wall_guard_stale_sec must be > 0"); }

    std::vector<LookaheadZone> lookahead_zones;
    std::string zone_reason;
    if (!parseLookaheadZones(lookahead_zone_spec, lookahead_zones, zone_reason)) {
      return rejectParameter("lookahead_scale_zones: " + zone_reason);
    }

    wheel_base_ = wheel_base;
    lookahead_gain_ = lookahead_gain;
    lookahead_min_distance_ = lookahead_min_distance;
    speed_proportional_gain_ = speed_proportional_gain;
    use_external_target_vel_ = use_external_target_vel;
    external_target_vel_ = external_target_vel;
    steering_tire_angle_gain_ = steering_tire_angle_gain;
    lookahead_cte_gain_ = lookahead_cte_gain;
    lookahead_slow_speed_ = lookahead_slow_speed;
    lookahead_curve_k_ = lookahead_curve_k;
    lookahead_slow_min_ = lookahead_slow_min;
    lookahead_slow_exp_ = lookahead_slow_exp;
    lookahead_overtake_scale_ = lookahead_overtake_scale;
    lookahead_zone_spec_ = lookahead_zone_spec;
    lookahead_zones_ = std::move(lookahead_zones);
    lookahead_curve_ahead_ = lookahead_curve_ahead;
    start_steer_speed_ = start_steer_speed;
    start_steer_limit_ = start_steer_limit;
    stuck_steer_free_speed_ = stuck_steer_free_speed;
    sat_accel_guard_ = sat_accel_guard;
    sat_steer_rad_ = sat_steer_rad;
    sat_accel_max_ = sat_accel_max;
    sat_guard_min_speed_ = sat_guard_min_speed;
    max_acceleration_ = max_acceleration;
    wall_guard_clamp_enable_ = wall_guard_clamp_enable;
    wall_guard_stale_sec_ = wall_guard_stale_sec;
  } catch (const std::exception & exception) {
    return rejectParameter(std::string("parameter update failed: ") + exception.what());
  } catch (...) {
    return rejectParameter("parameter update failed with an unknown exception");
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "accepted";
  return result;
}

bool SimplePurePursuit::subscribeMessageAvailable()
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
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
