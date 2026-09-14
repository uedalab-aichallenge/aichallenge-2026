#include "v2x_overtaker/stopped_pass_reachability.hpp"

#include <cassert>
#include <cstdio>

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
}  // namespace

int main()
{
  using v2x_overtaker::StoppedPassReachability;
  using v2x_overtaker::StoppedPassReachabilityInput;
  using v2x_overtaker::evaluateStoppedPassReachability;

  // D4 epoch 9264.955: 39.2m remains, but 24.2km/h is 0.3m short.
  StoppedPassReachabilityInput d4;
  d4.base_distance_m = 39.2;
  d4.free_width_m = 1.17;
  d4.required_width_m = 0.50;
  d4.band_center_m = 2.97;
  d4.ego_lateral_m = -0.52;
  d4.lateral_lag_m = 20.0;
  d4.lateral_rate_mps = 1.2;
  d4.ego_speed_mps = 24.2 / 3.6;
  const auto slowed = evaluateStoppedPassReachability(d4);
  require(slowed.state == StoppedPassReachability::kPassWithSpeedCap,
    "marginal lateral reachability must request a controlled cap");
  require(std::abs(slowed.v_lat_max_mps * 3.6 - 23.8) < 0.1,
    "D4 cap must be approximately 23.8km/h, not fixed 10.8km/h");

  d4.ego_lateral_m = 2.90;
  const auto in_band = evaluateStoppedPassReachability(d4);
  require(in_band.state == StoppedPassReachability::kPassNow,
    "in-band passage must not retain a stopped-car cap");

  d4.free_width_m = 0.49;
  const auto narrow = evaluateStoppedPassReachability(d4);
  require(narrow.state == StoppedPassReachability::kNoPass,
    "insufficient physical width must remain a stop case");

  d4.free_width_m = 1.17;
  d4.ego_lateral_m = -0.52;
  d4.base_distance_m = 19.9;
  const auto no_room_after_lag = evaluateStoppedPassReachability(d4);
  require(no_room_after_lag.state == StoppedPassReachability::kNoPass,
    "no distance after lateral lag must remain a stop case");
    // 止まったら二度と動けない不具合の退行検出。
  {
    auto mk = [](double base, double v) {
      StoppedPassReachabilityInput in;
      in.base_distance_m = base;
      in.free_width_m = 0.87;      // 実測値
      in.required_width_m = 0.50;
      in.band_center_m = 0.29;
      in.ego_lateral_m = 0.53;     // 横に 0.24m ずれるだけでよい
      in.lateral_lag_m = 1.0 + 2.7 * v;
      in.lateral_rate_mps = 1.2;
      in.ego_speed_mps = v;
      in.in_band_tolerance_m = 0.05;
      return in;
    };
    const auto stopped = evaluateStoppedPassReachability(mk(1.4, 0.0));
    std::printf("停止後の到達性: %s\n", stoppedPassReachabilityName(stopped.state));
    // **止まっていても通れる**こと。no_pass に戻ったらこの不具合の再発。
    assert(stopped.state != StoppedPassReachability::kNoPass);
  }

  return 0;
}
