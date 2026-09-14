#pragma once

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{

// 追突防止で「相手が自分の通る帯にいるか」を判定するときの横間隔。
inline double rearEndPredictedSeparation(
  double my_now, double my_want, double opp_lat,
  double gap, double closing, double offset_rate,
  double max_predict_sec)
{
  const double sep_min =
    std::min(std::abs(my_now - opp_lat), std::abs(my_want - opp_lat));
  if (!std::isfinite(my_now) || !std::isfinite(my_want) ||
      !std::isfinite(opp_lat) || !std::isfinite(gap) ||
      !std::isfinite(closing) || !std::isfinite(offset_rate) ||
      !std::isfinite(max_predict_sec))
  {
    return sep_min;                 // 壊れた値では従来どおり(安全側)
  }
  if (gap <= 0.0 || closing <= 0.0 || offset_rate <= 0.0 ||
      max_predict_sec <= 0.0)
  {
    return sep_min;
  }
  const double t = std::min(gap / closing, max_predict_sec);
  const double move_max = offset_rate * t;
  const double delta = my_want - my_now;
  const double moved = std::clamp(delta, -move_max, move_max);
  const double lat_at_meet = my_now + moved;
  // 相手から離れる目標なら従来(最小値)より緩く、相手へ寄る目標なら。
  return std::abs(lat_at_meet - opp_lat);
}

}  // namespace v2x_overtaker
