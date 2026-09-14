#pragma once

#include <cmath>
#include <limits>

namespace v2x_overtaker
{

enum class StoppedPassReachability
{
  kNoPass,
  kPassNow,
  kPassWithSpeedCap,
};

struct StoppedPassReachabilityInput
{
  double base_distance_m{0.0};
  double free_width_m{0.0};
  double required_width_m{0.0};
  double band_center_m{0.0};
  double ego_lateral_m{0.0};
  double lateral_lag_m{0.0};
  double lateral_rate_mps{0.0};
  double ego_speed_mps{0.0};
  double in_band_tolerance_m{0.15};
  double band_lo_m{0.0};
  double band_hi_m{0.0};
  bool use_band_edge{false};      // true で「帯に入るまで」で測る
  double band_edge_margin_m{0.15};   // 帯の内側にこれだけ入る
};

struct StoppedPassReachabilityResult
{
  StoppedPassReachability state{StoppedPassReachability::kNoPass};
  double delta_lateral_m{0.0};
  double needed_distance_m{std::numeric_limits<double>::infinity()};
  double v_lat_max_mps{0.0};
};

// The cap keeps lateral travel within base_distance_m after lateral_lag_m.
inline StoppedPassReachabilityResult evaluateStoppedPassReachability(
  const StoppedPassReachabilityInput & in)
{
  StoppedPassReachabilityResult out;
  const bool finite = std::isfinite(in.base_distance_m) &&
    std::isfinite(in.free_width_m) && std::isfinite(in.required_width_m) &&
    std::isfinite(in.band_center_m) && std::isfinite(in.ego_lateral_m) &&
    std::isfinite(in.lateral_lag_m) && std::isfinite(in.lateral_rate_mps) &&
    std::isfinite(in.ego_speed_mps) && std::isfinite(in.in_band_tolerance_m);
  if (!finite || in.base_distance_m < 0.0 || in.required_width_m < 0.0 ||
      in.lateral_lag_m < 0.0 || in.lateral_rate_mps <= 0.0 ||
      in.ego_speed_mps < 0.0 || in.in_band_tolerance_m < 0.0 ||
      in.free_width_m < in.required_width_m) {
    return out;
  }

  if (in.use_band_edge && in.band_hi_m > in.band_lo_m) {
    // 帯の内側 margin だけ入った区間へ、いちばん近い点まで。
    const double half = (in.band_hi_m - in.band_lo_m) * 0.5;
    const double m = std::min(in.band_edge_margin_m, half);
    const double lo = in.band_lo_m + m;
    const double hi = in.band_hi_m - m;
    const double tgt = std::min(std::max(in.ego_lateral_m, lo), hi);
    out.delta_lateral_m = std::abs(tgt - in.ego_lateral_m);
  } else {
    out.delta_lateral_m = std::abs(in.band_center_m - in.ego_lateral_m);
  }
  if (out.delta_lateral_m <= in.in_band_tolerance_m) {
    out.needed_distance_m = in.lateral_lag_m;
    out.v_lat_max_mps = in.ego_speed_mps;
    out.state = StoppedPassReachability::kPassNow;
    return out;
  }

  out.needed_distance_m = in.lateral_lag_m +
    out.delta_lateral_m * in.ego_speed_mps / in.lateral_rate_mps;
  if (in.base_distance_m >= out.needed_distance_m) {
    out.v_lat_max_mps = in.ego_speed_mps;
    out.state = StoppedPassReachability::kPassNow;
    return out;
  }

  const double travel_distance_m = in.base_distance_m - in.lateral_lag_m;
  if (travel_distance_m > 0.0) {
    out.v_lat_max_mps = in.lateral_rate_mps * travel_distance_m /
      out.delta_lateral_m;
    if (std::isfinite(out.v_lat_max_mps) && out.v_lat_max_mps > 0.0) {
      // 処理内容を示す。
      out.state = (out.v_lat_max_mps < in.ego_speed_mps)
                    ? StoppedPassReachability::kPassWithSpeedCap
                    : StoppedPassReachability::kPassNow;
    }
  }
  return out;
}

inline const char * stoppedPassReachabilityName(StoppedPassReachability state)
{
  switch (state) {
    case StoppedPassReachability::kNoPass: return "no_pass";
    case StoppedPassReachability::kPassNow: return "pass_now";
    case StoppedPassReachability::kPassWithSpeedCap: return "pass_with_speed_cap";
  }
  return "no_pass";
}

}  // namespace v2x_overtaker
