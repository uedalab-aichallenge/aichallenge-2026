#pragma once

#include <cmath>
#include <string>

namespace v2x_overtaker
{

inline bool plannedSpotReady(
  bool enabled, bool valid, bool same_target, double distance, double dynamic_gate)
{
  return enabled && valid && same_target && std::isfinite(distance) &&
    std::isfinite(dynamic_gate) && distance >= 0.0 && dynamic_gate >= 0.0 &&
    distance <= dynamic_gate;
}

// 通常は normal_gap、計画済みの抜きどころか公式レーンなら entry_floor 以上の車間で開始を許す。
inline bool overtakeStartGapOk(
  bool already_active, double gap, double normal_gap,
  bool planned_spot_ready, bool official_lane_ready, double entry_floor)
{
  if (already_active) { return true; }
  if (!std::isfinite(gap) || !std::isfinite(normal_gap) ||
    !std::isfinite(entry_floor) || gap < 0.0 || normal_gap < 0.0 || entry_floor < 0.0)
  {
    return false;
  }
  return gap >= normal_gap ||
    ((planned_spot_ready || official_lane_ready) && gap >= entry_floor);
}

// 今周期の許可対象がいまの前方車と一致するときだけ追い越しの開始を認める。
inline bool overtakeAttemptStartAuthorized(
  bool already_active, bool moving_out, bool overtake_intent,
  bool require_intent, const std::string & blocker,
  const std::string & authorized_target)
{
  if (already_active) { return true; }
  return moving_out && (overtake_intent || !require_intent) &&
    !blocker.empty() && authorized_target == blocker;
}

}  // namespace v2x_overtaker
