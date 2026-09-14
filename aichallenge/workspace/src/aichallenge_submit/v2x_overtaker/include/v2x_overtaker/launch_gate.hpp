#pragma once

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{

// 自車か他車が閾値を超えて動いたら物理的な発進とみなす。
inline bool launchMotionObserved(
  double ego_speed, double fastest_opponent_speed, double threshold)
{
  if (!std::isfinite(ego_speed) || !std::isfinite(fastest_opponent_speed) ||
    !std::isfinite(threshold) || threshold < 0.0)
  {
    return false;
  }
  return std::max(std::abs(ego_speed), std::abs(fastest_opponent_speed)) > threshold;
}

// 発進前と発進直後の猶予中は停止車の判定を抑える。
inline bool suppressStoppedCarsAtLaunch(
  double motion_since, double now, double grace)
{
  if (!std::isfinite(now) || !std::isfinite(grace) || grace < 0.0) { return false; }
  if (motion_since < 0.0) { return true; }
  return now - motion_since < grace;
}

// 発進前か、発進から duration 秒以内なら発進時の区間とする。
inline bool launchWindowActive(double motion_since, double now, double duration)
{
  if (!std::isfinite(now) || !std::isfinite(duration) || duration <= 0.0) {
    return false;
  }
  return motion_since < 0.0 || now - motion_since < duration;
}

}  // namespace v2x_overtaker
