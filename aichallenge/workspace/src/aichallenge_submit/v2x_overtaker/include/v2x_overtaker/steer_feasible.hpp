// 舵の限界から「その車間で、その横ずれを作れるか」を出す。
//
// 2026-09-18 ユーザー指示。それまで「横へ避けるから減速しない」の判断は、横移動の速さの
// 見積り(offset_rate 1.2m/s)だけで行っており、舵の限界も今の速度も見ていなかった。
// 実測では舵は 18度(0.31rad)で飽和し、最小旋回半径は ホイールベース1.087/tan(18deg)=3.35m。
#ifndef V2X_OVERTAKER__STEER_FEASIBLE_HPP_
#define V2X_OVERTAKER__STEER_FEASIBLE_HPP_

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{
struct SteerFeasibleInput
{
  double lateral_need_m{0.0};    // これから作るべき横ずれ[m]
  double distance_m{0.0};        // それを作れる縦距離[m](車間など)
  double wheel_base_m{1.087};
  double max_steer_rad{0.31};    // 実測の飽和値
  double ay_max{23.0};           // 横加速度の上限[m/s^2](実測 23.2)
  double ay_use{0.6};            // そのうち使う割合(余裕)
  double v_floor{1.0};           // 下限[m/s]。これ以下は要求しない
};

struct SteerFeasibleResult
{
  bool need_move{false};      // 横ずれが要るか
  bool possible{false};       // その距離で作れるか(舵の限界の範囲で)
  double radius_need_m{0.0};  // 要る旋回半径[m]
  double radius_min_m{0.0};   // 舵の限界から決まる最小旋回半径[m]
  double v_max_mps{-1.0};     // その横ずれを作れる上限速度[m/s]。制限不要なら負
};

// 距離 L の間に横へ d ずらすのに要る旋回半径は、円弧の近似で R ≒ L^2/(8d)。
// その半径を曲がれる速度は v <= sqrt(ay * R)。舵の限界で R が足りなければ不可能。
inline SteerFeasibleResult steerFeasible(const SteerFeasibleInput & in)
{
  SteerFeasibleResult out;
  const double d = in.lateral_need_m;
  const double L = in.distance_m;
  if (!std::isfinite(d) || !std::isfinite(L) || d <= 0.01) { return out; }
  out.need_move = true;
  if (L <= 0.05) { return out; }                 // 距離が無い = 作れない
  out.radius_need_m = (L * L) / (8.0 * d);
  out.radius_min_m = (std::tan(in.max_steer_rad) > 1e-6)
                       ? in.wheel_base_m / std::tan(in.max_steer_rad) : 1e9;
  if (out.radius_need_m < out.radius_min_m) { return out; }   // 舵を全部使っても曲がれない
  out.possible = true;
  const double ay = std::max(in.ay_max * std::clamp(in.ay_use, 0.05, 1.0), 0.1);
  out.v_max_mps = std::sqrt(ay * out.radius_need_m);
  if (out.v_max_mps < in.v_floor) { out.v_max_mps = in.v_floor; }
  return out;
}
}  // namespace v2x_overtaker
#endif  // V2X_OVERTAKER__STEER_FEASIBLE_HPP_
