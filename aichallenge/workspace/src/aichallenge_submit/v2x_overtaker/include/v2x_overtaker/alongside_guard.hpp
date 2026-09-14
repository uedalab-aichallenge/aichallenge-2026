#pragma once

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{

inline double physicalPassSeparation(double configured, double physical_minimum)
{
  if (!std::isfinite(configured) || !std::isfinite(physical_minimum)) { return 1e9; }
  return std::max(configured, physical_minimum);
}

// 縦に重なる相手に対し、横目標を安全な横間隔の外側までに制限する。
inline double alongsideSafeLimit(
  bool opponent_on_left, double ego_lat, double opponent_lat, double need)
{
  if (!std::isfinite(ego_lat) || !std::isfinite(opponent_lat) ||
    !std::isfinite(need) || need < 0.0)
  {
    return opponent_on_left ? -1e9 : 1e9;
  }
  return opponent_on_left ? std::min(ego_lat, opponent_lat - need)
                          : std::max(ego_lat, opponent_lat + need);
}

}  // namespace v2x_overtaker
