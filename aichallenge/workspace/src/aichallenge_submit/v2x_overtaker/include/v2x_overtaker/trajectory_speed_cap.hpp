#pragma once

#include <cmath>
#include <cstddef>

namespace v2x_overtaker
{

// 軌道の全点に速度上限を掛ける。
template<typename TrajectoryT>
std::size_t applyTrajectorySpeedCap(TrajectoryT & trajectory, double cap)
{
// 負は制限なし、非有限値は無視する。
  if (!std::isfinite(cap) || cap < 0.0) {
    return 0;
  }

  for (auto & point : trajectory.points) {
    const double velocity = point.longitudinal_velocity_mps;
    point.longitudinal_velocity_mps = static_cast<decltype(point.longitudinal_velocity_mps)>(
      std::isfinite(velocity) && velocity <= cap ? velocity : cap);
  }
  return trajectory.points.size();
}

}  // namespace v2x_overtaker
