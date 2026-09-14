#include "v2x_overtaker/trajectory_speed_cap.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace
{

struct Point
{
  double x;
  double y;
  float longitudinal_velocity_mps;
};

struct Trajectory
{
  std::vector<Point> points;
};

std::size_t nearest(const Trajectory & trajectory, double x, double y)
{
  std::size_t best = 0;
  double best_squared_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const double dx = trajectory.points[i].x - x;
    const double dy = trajectory.points[i].y - y;
    const double squared_distance = dx * dx + dy * dy;
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best = i;
    }
  }
  return best;
}

bool near(double actual, double expected)
{
  return std::abs(actual - expected) < 1e-6;
}

}  // namespace

int main()
{
  using v2x_overtaker::applyTrajectorySpeedCap;

// pure pursuit が選ぶ点を含む全点に上限が掛かる。
  Trajectory deformed{{
    {0.0, 0.1, 11.67F},
    {0.0, 2.0, 11.67F},
    {1.0, 2.0, 8.0F},
  }};
  const std::size_t pp_index = nearest(deformed, 0.0, 0.0);
  if (pp_index != 0) { return 1; }
  if (applyTrajectorySpeedCap(deformed, 0.0) != deformed.points.size()) { return 2; }
  if (!near(deformed.points[pp_index].longitudinal_velocity_mps, 0.0)) { return 3; }
  for (const auto & point : deformed.points) {
    if (!std::isfinite(point.longitudinal_velocity_mps) ||
        point.longitudinal_velocity_mps > 0.0F) { return 4; }
  }

// 上限より遅い点はそのまま、非有限の速度は上限にする。
  Trajectory mixed{{
    {0.0, 0.0, 2.0F},
    {1.0, 0.0, 7.0F},
    {2.0, 0.0, std::numeric_limits<float>::quiet_NaN()},
    {3.0, 0.0, std::numeric_limits<float>::infinity()},
  }};
  if (applyTrajectorySpeedCap(mixed, 5.0) != mixed.points.size()) { return 5; }
  if (!near(mixed.points[0].longitudinal_velocity_mps, 2.0) ||
      !near(mixed.points[1].longitudinal_velocity_mps, 5.0) ||
      !near(mixed.points[2].longitudinal_velocity_mps, 5.0) ||
      !near(mixed.points[3].longitudinal_velocity_mps, 5.0)) { return 6; }

// 負や NaN の上限では何も変えない。
  Trajectory disabled{{{0.0, 0.0, 3.0F}, {1.0, 0.0, 9.0F}}};
  if (applyTrajectorySpeedCap(disabled, -1.0) != 0 ||
      !near(disabled.points[0].longitudinal_velocity_mps, 3.0) ||
      !near(disabled.points[1].longitudinal_velocity_mps, 9.0)) { return 7; }
  if (applyTrajectorySpeedCap(disabled, std::numeric_limits<double>::quiet_NaN()) != 0 ||
      !near(disabled.points[0].longitudinal_velocity_mps, 3.0) ||
      !near(disabled.points[1].longitudinal_velocity_mps, 9.0)) { return 8; }

  return 0;
}
