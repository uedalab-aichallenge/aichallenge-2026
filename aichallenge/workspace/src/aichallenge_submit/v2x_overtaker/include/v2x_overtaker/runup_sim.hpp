// 助走の前向きシミュレーション。
#ifndef V2X_OVERTAKER__RUNUP_SIM_HPP_
#define V2X_OVERTAKER__RUNUP_SIM_HPP_

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{

struct RunupSimIn
{
  double v_now{0.0};        // 自車速度[m/s]
  double v_opp{0.0};        // 相手の現在速度[m/s](予測が無いときの代替)
  // 相手の将来速度[m/s]。i 番目 = いまから i*dt 秒後。空なら v_opp を使う。
  const double * opp_v{nullptr};
  int opp_n{0};
  double gap{0.0};          // いまの車間[m](経路上の最近傍点どうし)
  double body{0.0};         // 車体長[m](gap から引いて実効の車間にする)
  double dist{0.0};         // 目標地点(レーン入口)までの距離[m]
  double a_max{3.2};        // 出せる加速度[m/s^2]
  double v_cap{10.0};       // 順位ハンデ等の速度上限[m/s]
  // 追突防止の式のパラメータ
  double rear_margin{3.5};
  double rear_time{0.15};
  double brake_a{0.55};
  double dt{0.1};
  double horizon{8.0};
};

struct RunupSimOut
{
  bool   valid{false};
  double v_at_dist{0.0};    // 目標地点に着いたときの速度[m/s]
  double min_gap{0.0};      // その間の最小車間[m]
  double t_at_dist{0.0};    // 到達までの時間[s]
  bool   capped{false};     // 途中で追突防止に頭打ちされたか
};

// 追突防止が許す速度。preventRearEnd と同じ式。
inline double rearEndAllow(
  double gap_eff, double v, double v_opp,
  double rear_margin, double rear_time, double brake_a)
{
  const double room = gap_eff - rear_margin - v * rear_time;
  if (room <= 0.0) { return std::max(v_opp, 0.0); }
  return std::sqrt(2.0 * std::max(brake_a, 1e-3) * room) + v_opp;
}

inline RunupSimOut simulateRunup(const RunupSimIn & in)
{
  RunupSimOut out;
  if (!std::isfinite(in.v_now) || !std::isfinite(in.gap) ||
      !std::isfinite(in.dist) || in.dist < 0.0 || in.dt <= 0.0)
  {
    return out;
  }
  double v = std::max(in.v_now, 0.0);
  double g = in.gap;
  double s = 0.0;
  double t = 0.0;
  out.min_gap = g - in.body;
  const int steps = static_cast<int>(in.horizon / in.dt) + 1;
  for (int k = 0; k < steps; ++k) {
    const double gap_eff = g - in.body;
    out.min_gap = std::min(out.min_gap, gap_eff);
    // 相手の将来速度。予測列があればそれを使う(無ければ現在値)。
    const double vo = (in.opp_v && in.opp_n > 0)
                        ? in.opp_v[std::min(k, in.opp_n - 1)]
                        : in.v_opp;
    const double v_allow = rearEndAllow(
      gap_eff, v, vo, in.rear_margin, in.rear_time, in.brake_a);
    // 追突防止の上限を超えない範囲でだけ加速する
    const double v_want = std::min(in.v_cap, v_allow);
    if (v < v_want) {
      v = std::min(v_want, v + in.a_max * in.dt);
    } else if (v > v_want) {
      out.capped = true;
      v = v_want;                      // 上限に張り付く(減速はここで表現)
    }
    s += v * in.dt;
    g -= (v - vo) * in.dt;
    t += in.dt;
    if (s >= in.dist) {
      out.valid = true;
      out.v_at_dist = v;
      out.t_at_dist = t;
      return out;
    }
  }
  out.valid = true;                    // 地平まで届かなかった。そのときの速度を返す
  out.v_at_dist = v;
  out.t_at_dist = t;
  return out;
}

}  // namespace v2x_overtaker

#endif  // V2X_OVERTAKER__RUNUP_SIM_HPP_
