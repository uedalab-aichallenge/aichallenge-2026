#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace v2x_overtaker
{

// 抜く側を「いま空いているほう」ではなく「**抜き切る地点で**空いているほう」で決める。

struct PassSideSample
{
  double s{0.0};        // 自車からの距離[m]
  double lo{0.0};       // 可動域の下限(負側)[m]
  double hi{0.0};       // 可動域の上限(正側)[m]
  double opp_lat{0.0};  // その地点での相手の予測横位置[m]
  bool opp_known{false};  // 相手の横位置が分かっているか
  // その地点の曲率半径[m]。0以下なら不明(判定しない)。
  // 「その側を通る経路を、その速度で曲がれるか」の判定に使う。
  double radius_m{0.0};
};

struct PassSideInput
{
  std::vector<PassSideSample> samples;
  double pass_dist{0.0};   // 抜き切るのに要る距離[m]
  double need_sep{1.66};   // 相手との必要横間隔[m]
  // 抜き切る地点の手前どこから評価するか(pass_dist に対する比)。
  // 0.5 なら「後半半分で通れること」を要求する。
  double window_ratio{0.5};

  bool plan_mode{false};      // true で「早く通せるほう」を選ぶ
  double y_now{0.0};          // 自車のいまの横位置[m]
  double v_ego{8.0};          // 自車速度[m/s](窓までの所要時間に使う)
  double lat_rate{1.2};       // 実現できる横移動レート[m/s](実測値)
  double reach_gain{1.0};     // 到達時間に掛ける安全率(1.0 で余裕なし)
  double room_margin{0.0};    // これ以上空いていれば「通れる」とみなす[m]
  double ay_max{0.0};
  // 曲率の符号。+1 なら左カーブ(左がイン)、-1 なら右カーブ、0 なら直線/不明。
  int curve_sign{0};

  double zone_end_s{-1.0};

  // plan_mode で窓を短縮するのをやめる。
  bool plan_full_scale{true};
};

struct PassSideResult
{
  int side{0};          // +1 = 左 / -1 = 右 / 0 = 決められない
  double room_left{-1e9};   // 抜き切り窓での左の最小空き[m]
  double room_right{-1e9};  // 同 右
  bool valid{false};
  // どの窓で判定したのかを残す。
  double s_from{0.0};
  double s_to{0.0};
  int used{0};
  double scale{1.0};
  // --- plan_mode の出力 ---
  double start_s{-1.0};   // 抜き始める地点(窓の入口)までの距離[m]
  double y_target{0.0};   // その側で取るべき横位置[m]
  bool reach_ok{false};   // 窓の入口までにその横位置へ寄れるか
  // 通せると判定した側の内訳(監査用)。
  bool feas_left{false};
  bool feas_right{false};
  // その側で「窓の全域にわたって取り続けられる」横位置の区間。
  // 区間が空(lo > hi)ならその側では一本の線で通しきれない。
  double band_left_lo{0.0}, band_left_hi{0.0};
  double band_right_lo{0.0}, band_right_hi{0.0};
  double earliest_left{-1.0};
  double earliest_right{-1.0};
  int rej_room_l{0}, rej_band_l{0}, rej_reach_l{0}, rej_ay_l{0};
  int rej_room_r{0}, rej_band_r{0}, rej_reach_r{0}, rej_ay_r{0};
};

// 抜き切り窓(pass_dist の手前 window_ratio 〜 pass_dist)で、
// 左右それぞれ「相手を避けて自車中心を置ける幅」の最小値を出し、大きいほうを返す。
//
// 相手の横位置が分からない地点は、走行ラインを境にした左右の広さで測る。
// いずれにせよ楽観側なので、呼び出し側は別途 fit / 幅 の判定を通すこと。
inline PassSideResult passCompletionSide(const PassSideInput & in);

// 窓を滑らせて「通せる場所」を探す。
inline PassSideResult passCompletionSideSearch(
  const PassSideInput & in, double search_limit_m, double step_m,
  double & found_at_m)
{
  found_at_m = -1.0;
  const double step = std::max(step_m, 1.0);
  const double limit = (std::isfinite(search_limit_m) && search_limit_m > 0.0)
                         ? search_limit_m : 0.0;

  // 処理内容を示す。
  PassSideResult best;
  PassSideResult tally;   // 側ごとの内訳。勝った窓と混ざらないよう別に持つ。
  double best_score = -1e9;
  double best_at = -1.0;
  // 縮尺を 0.5 まで許すのをやめた。
  const double scales_all[] = {1.0, 0.85};
  const double scales_full[] = {1.0};
  const bool full_only = in.plan_mode && in.plan_full_scale;
  const double * scales = full_only ? scales_full : scales_all;
  const std::size_t n_scales = full_only ? 1u : 2u;
  for (std::size_t si = 0; si < n_scales; ++si) {
    const double sc = scales[si];
    for (double shift = 0.0; shift <= limit; shift += step) {
      // 窓の**後端**が、いまいる直線を出るなら採らない。
      if (in.zone_end_s > 0.0 && shift + in.pass_dist * sc > in.zone_end_s) { break; }
      PassSideInput cand = in;
      cand.pass_dist = in.pass_dist * sc;
      for (auto & sm : cand.samples) { sm.s -= shift; }
      const auto r = passCompletionSide(cand);
      if (!in.plan_mode) {
        if (r.side == 0) { continue; }
        const double room = std::max(r.room_left, r.room_right);
        // 近さの優遇。10m 先へ行くごとに 0.10m ぶん割り引く。
        // 空きの差が 0.1m 未満なら近いほうを採る、程度の弱い重み。
        const double score = room - shift * 0.01;
        if (score > best_score) {
          best_score = score;
          best = r;
          best.scale = sc;
          best_at = shift;
        }
        continue;
      }
      // --- plan_mode: 「通せる窓のうち最も早いもの」を選ぶ ---
      //
      // 通れるか(room > margin)は二値。通れる側どうしを幅で競わせない。
      // 競うのは **抜き始める地点の早さ**。同じ早さなら幅の広いほうを採る。
      if (!r.valid) { continue; }
      const double win_start = shift + r.s_from;   // 窓の入口までの距離[m]
      for (int sd = +1; sd >= -1; sd -= 2) {
        const double room = (sd > 0) ? r.room_left : r.room_right;
        int & rj_room  = (sd > 0) ? tally.rej_room_l  : tally.rej_room_r;
        int & rj_band  = (sd > 0) ? tally.rej_band_l  : tally.rej_band_r;
        int & rj_reach = (sd > 0) ? tally.rej_reach_l : tally.rej_reach_r;
        int & rj_ay    = (sd > 0) ? tally.rej_ay_l    : tally.rej_ay_r;
        if (!(room > in.room_margin)) { ++rj_room; continue; }
        const double b_lo = (sd > 0) ? r.band_left_lo : r.band_right_lo;
        const double b_hi = (sd > 0) ? r.band_left_hi : r.band_right_hi;
        if (!(b_hi > b_lo)) { ++rj_band; continue; }   // 一本の線では通しきれない
        // その側で取るべき横位置。いまの位置にいちばん近い点を取る。
        const double y_tgt = std::clamp(in.y_now, b_lo, b_hi);
        // --- 経路の追従可能性(横加速度) ---
        // その横位置を保って走ると曲率半径は R' = R + (イン側なら -|y|, アウト側なら +|y|)。
        // 必要な横加速度 v^2/R' が限界を超えるなら、その側は「通れる」としない。
        if (in.ay_max > 0.0) {
          bool ay_ok = true;
          for (const auto & sm2 : cand.samples) {
            if (sm2.s < r.s_from || sm2.s > r.s_to) { continue; }
            if (!(sm2.radius_m > 0.1)) { continue; }   // 直線/不明は判定しない
            // in.curve_sign と同じ側へ寄るとイン(半径が縮む)。
            const double inward = (in.curve_sign != 0 &&
                                   ((in.curve_sign > 0) == (y_tgt > 0.0))) ? -1.0 : +1.0;
            const double r_eff = sm2.radius_m + inward * std::abs(y_tgt);
            if (r_eff < 0.5) { ay_ok = false; break; }
            const double ay = in.v_ego * in.v_ego / r_eff;
            if (ay > in.ay_max) { ay_ok = false; break; }
          }
          if (!ay_ok) { ++rj_ay; continue; }   // この側・この窓では曲がれない
        }
        // 到達性。窓の入口までに寄り切れるか。
        // 使える時間 = 窓の入口までの距離 / 自車速度
        // 要る時間   = 横に動く量 / 横移動レート
        const double v = std::max(in.v_ego, 1.0);
        const double rate = std::max(in.lat_rate, 0.05);
        const double t_have = win_start / v;
        const double t_need = std::abs(y_tgt - in.y_now) / rate * in.reach_gain;
        const bool reach = (t_need <= t_have);
        if (!reach) { ++rj_reach; continue; }
        // その側で「通せる」と判定できた最も早い窓を控える(勝敗とは無関係)。
        double & early = (sd > 0) ? tally.earliest_left : tally.earliest_right;
        if (early < 0.0 || win_start < early) { early = win_start; }
        // 早いほど良い。同点(0.5m 以内)なら幅の広いほうを採る。
        const double score = -win_start + std::min(room, 2.0) * 0.02;
        if (score > best_score) {
          best_score = score;
          best = r;
          best.scale = sc;
          best.side = sd;
          best.start_s = win_start;
          best.y_target = y_tgt;
          best.reach_ok = true;
          best.feas_left = (r.room_left > in.room_margin);
          best.feas_right = (r.room_right > in.room_margin);
          best_at = shift;
        }
      }
    }
  }
  // 処理内容を示す。
  if (in.plan_mode && best.side != 0) {
    const double room = (best.side > 0) ? best.room_left : best.room_right;
    if (!(room > in.room_margin)) {
      best.side = 0;
      best.reach_ok = false;
    }
  }
  auto carry = [&tally](PassSideResult & x) {
    x.earliest_left = tally.earliest_left;   x.earliest_right = tally.earliest_right;
    x.rej_room_l = tally.rej_room_l; x.rej_band_l = tally.rej_band_l;
    x.rej_reach_l = tally.rej_reach_l; x.rej_ay_l = tally.rej_ay_l;
    x.rej_room_r = tally.rej_room_r; x.rej_band_r = tally.rej_band_r;
    x.rej_reach_r = tally.rej_reach_r; x.rej_ay_r = tally.rej_ay_r;
  };
  if (best.side != 0) { carry(best); found_at_m = best_at; return best; }
  // 計画が成立しなかったときのフォールバック。
  auto r = passCompletionSide(in);
  if (in.plan_mode) {
    r.side = 0;                 // 広いほうを答えにしない
    r.reach_ok = false;
  }
  carry(r);
  return r;
}

inline PassSideResult passCompletionSide(const PassSideInput & in)
{
  PassSideResult r;
  if (in.samples.empty() || !std::isfinite(in.pass_dist) || in.pass_dist <= 0.0) {
    return r;
  }
  const double ratio = std::clamp(in.window_ratio, 0.0, 1.0);
  const double s_from = in.pass_dist * ratio;
  const double s_to = in.pass_dist;

  double min_left = 1e9;
  double min_right = 1e9;
  double band_l_lo = -1e9, band_l_hi = 1e9;
  double band_r_lo = -1e9, band_r_hi = 1e9;
  double last_s = -1e9;   // 直前の標本位置(帯を広げる量の計算に使う)
  std::size_t used = 0;
  for (const auto & sm : in.samples) {
    if (!std::isfinite(sm.s) || sm.s < s_from || sm.s > s_to) { continue; }
    if (!std::isfinite(sm.lo) || !std::isfinite(sm.hi) || sm.hi < sm.lo) { continue; }
    ++used;
    double left_room = 0.0;
    double right_room = 0.0;
    if (sm.opp_known) {
      // 左側(正)に自車中心を置ける幅。相手の上側だけ。
      const double left_floor = std::max(sm.lo, sm.opp_lat + in.need_sep);
      left_room = sm.hi - left_floor;
      // 右側(負)に置ける幅。相手の下側だけ。
      const double right_ceil = std::min(sm.hi, sm.opp_lat - in.need_sep);
      right_room = right_ceil - sm.lo;
    } else {
      // 相手の横位置が分からない地点。可動域をそのまま左右に当てると
      // **両側とも「全幅」になって差が消える**ので、走行ラインを境に
      // どちら側がどれだけ広いかで測る(これが本来の「空いている側」)。
      left_room = sm.hi;
      right_room = -sm.lo;
    }
    min_left = std::min(min_left, left_room);
    min_right = std::min(min_right, right_room);
    // 帯を「一本の固定線」から。
    const double ds = (last_s > -1e8) ? std::max(sm.s - last_s, 0.0) : 0.0;
    const double move = in.lat_rate * (ds / std::max(in.v_ego, 1.0));
    if (band_l_lo > -1e8) { band_l_lo -= move; band_l_hi += move; }
    if (band_r_lo > -1e8) { band_r_lo -= move; band_r_hi += move; }
    last_s = sm.s;
    if (sm.opp_known) {
      band_l_lo = std::max(band_l_lo, std::max(sm.lo, sm.opp_lat + in.need_sep));
      band_l_hi = std::min(band_l_hi, sm.hi);
      band_r_lo = std::max(band_r_lo, sm.lo);
      band_r_hi = std::min(band_r_hi, std::min(sm.hi, sm.opp_lat - in.need_sep));
    } else {
      band_l_lo = std::max(band_l_lo, std::max(sm.lo, 0.0));
      band_l_hi = std::min(band_l_hi, sm.hi);
      band_r_lo = std::max(band_r_lo, sm.lo);
      band_r_hi = std::min(band_r_hi, std::min(sm.hi, 0.0));
    }
  }
  r.s_from = s_from;
  r.s_to = s_to;
  r.used = static_cast<int>(used);
  if (used == 0) { return r; }

  r.valid = true;
  r.room_left = min_left;
  r.room_right = min_right;
  r.band_left_lo = band_l_lo;  r.band_left_hi = band_l_hi;
  r.band_right_lo = band_r_lo; r.band_right_hi = band_r_hi;
  if (min_left <= 0.0 && min_right <= 0.0) { return r; }
  // 明確な差があるときだけ側を返す。僅差で毎周期反転させない。
  constexpr double kHyst = 0.20;   // [m]
  if (min_left > min_right + kHyst) { r.side = +1; }
  else if (min_right > min_left + kHyst) { r.side = -1; }
  else { r.side = (min_left >= min_right) ? +1 : -1; }
  return r;
}

}  // namespace v2x_overtaker
