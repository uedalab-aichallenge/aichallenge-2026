#pragma once

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{

// 横位置が実現するまでの「遅れ」を距離[m]で返す。

inline double lateralLagDistance(
  double ego_speed_mps, double lag_sec, double lag_base_m, double lag_max_m)
{
  if (!std::isfinite(ego_speed_mps) || !std::isfinite(lag_sec) ||
      !std::isfinite(lag_base_m))
  {
    return lag_max_m;   // 壊れた値では安全側(最大)に倒す
  }
  const double v = std::max(ego_speed_mps, 0.0);
  const double d = std::max(lag_base_m, 0.0) + std::max(lag_sec, 0.0) * v;
  if (!std::isfinite(lag_max_m) || lag_max_m <= 0.0) { return d; }
  return std::min(d, lag_max_m);
}

// 追い越しの準備(横へ出始める)を開始してよい車間[m]。
inline double prepareGateDistance(
  double ego_speed_mps, double lateral_move_m, double offset_rate_mps,
  double lag_sec, double lag_base_m, double lag_max_m,
  double floor_m, double margin)
{
  const double v = std::max(ego_speed_mps, 0.0);
  const double rate = std::max(offset_rate_mps, 0.1);
  const double move = std::max(lateral_move_m, 0.0);
  const double need =
    lateralLagDistance(v, lag_sec, lag_base_m, lag_max_m) + move * v / rate;
  const double m = std::isfinite(margin) && margin > 0.0 ? margin : 1.0;
  const double gate = need * m;
  if (!std::isfinite(gate)) { return floor_m; }
  return std::max(floor_m, gate);
}

}  // namespace v2x_overtaker
