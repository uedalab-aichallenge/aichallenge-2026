#ifndef STUCK_RECOVERY_CONTROLLER__PHASE_PATH_HPP_
#define STUCK_RECOVERY_CONTROLLER__PHASE_PATH_HPP_

#include "stuck_recovery_controller/recovery_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace recovery
{

// Odometry distance by itself cannot prove that a phase was executed in its
// requested direction.  In particular, forward inertia after selecting a
// short reverse phase used to satisfy that reverse phase.  Project each motion
// sample onto the vehicle's mid-heading and give it the phase direction's sign.
// Callers intentionally keep negative accumulated progress: the vehicle must
// first undo travel in the wrong direction before it can complete the phase.
inline double directedTravelIncrement(
  const Pose & previous, const Pose & current, bool phase_forward)
{
  double dyaw = current.yaw - previous.yaw;
  while (dyaw > M_PI) { dyaw -= 2.0 * M_PI; }
  while (dyaw < -M_PI) { dyaw += 2.0 * M_PI; }
  const double mid_yaw = previous.yaw + 0.5 * dyaw;
  const double longitudinal =
    (current.x - previous.x) * std::cos(mid_yaw) +
    (current.y - previous.y) * std::sin(mid_yaw);
  return phase_forward ? longitudinal : -longitudinal;
}

// 後退経路は index が増える向きにも車両が後退する。末尾2点の yaw 差は既に
// その走行方向の符号を持つため、正の弧長で割った曲率を再度反転しない。
inline std::vector<Pose> makeReverseTail(
  const Pose & last, const std::optional<Pose> & previous, double length, double step)
{
  std::vector<Pose> tail;
  if (length <= 0.0 || step <= 0.0) { return tail; }

  double curvature = 0.0;
  if (previous) {
    const double ds = std::hypot(last.x - previous->x, last.y - previous->y);
    if (ds > 1e-6) {
      double dyaw = last.yaw - previous->yaw;
      while (dyaw > M_PI) { dyaw -= 2.0 * M_PI; }
      while (dyaw < -M_PI) { dyaw += 2.0 * M_PI; }
      curvature = dyaw / ds;
    }
  }

  Pose pose = last;
  for (double run = 0.0; run < length; run += step) {
    const double ds = std::min(step, length - run);
    const double dyaw = ds * curvature;
    const double mid = pose.yaw + dyaw * 0.5;
    pose.x -= ds * std::cos(mid);
    pose.y -= ds * std::sin(mid);
    pose.yaw += dyaw;
    tail.push_back(pose);
  }
  return tail;
}

// 刻み幅を仮定せず、現在区間の経路距離で切り詰める。
inline bool truncatePhasePath(Plan & plan, std::size_t phase_index, double length)
{
  if (phase_index >= plan.phases.size() || length <= 0.0) { return false; }
  auto & phase = plan.phases[phase_index];
  if (phase.path_begin >= phase.path_end || phase.path_end > plan.path.size() ||
      phase.path_end - phase.path_begin < 2)
  {
    return false;
  }
  const auto end = phase.path_end;
  std::size_t keep = phase.path_begin + 1;
  double travelled = 0.0;
  for (std::size_t i = keep; i < end; ++i) {
    travelled += std::hypot(
      plan.path[i].x - plan.path[i - 1].x, plan.path[i].y - plan.path[i - 1].y);
    keep = i + 1;
    if (travelled >= length - 1e-6) { break; }
  }
  phase.length = length;
  phase.path_end = keep;
  plan.phases.resize(phase_index + 1);
  plan.path.resize(keep);
  plan.rev_points = std::min(plan.rev_points, keep);
  return true;
}

// 進捗は現在の操作区間内で単調。次の区間の交差点を最近傍に選ばない。
inline std::optional<std::size_t> nearestInPhase(
  const Plan & plan, std::size_t phase_index, std::size_t previous, const Pose & pose)
{
  if (phase_index >= plan.phases.size()) { return std::nullopt; }
  const auto & phase = plan.phases[phase_index];
  if (phase.path_begin >= phase.path_end || phase.path_end > plan.path.size()) {
    return std::nullopt;
  }
  // 区間が切り替わった直後など、前回値が現在区間の外なら区間先頭へ戻す。
  // 終端へ clamp すると新しい区間を未走行のまま使い切った扱いになる。
  const auto begin =
    (previous >= phase.path_begin && previous < phase.path_end) ? previous : phase.path_begin;
  std::optional<std::size_t> best;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = begin; i < phase.path_end; ++i) {
    const double d = std::hypot(plan.path[i].x - pose.x, plan.path[i].y - pose.y);
    if (d < distance) { distance = d; best = i; }
  }
  return best;
}

}  // namespace recovery

#endif  // STUCK_RECOVERY_CONTROLLER__PHASE_PATH_HPP_
