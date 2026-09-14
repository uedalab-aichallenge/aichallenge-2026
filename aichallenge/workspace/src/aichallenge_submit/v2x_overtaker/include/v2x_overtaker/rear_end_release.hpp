#pragma once

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{

// 追突防止(rear-end guard)を解除してよいかの判定。

struct RearEndReleaseInput
{
  double sep_now{0.0};        // いまの横間隔[m](絶対値で渡す)
  double sep_rate{0.0};       // 横間隔の変化率[m/s](正で開きつつある)
  double gap{0.0};            // 縦の車間[m]
  double closing{0.0};        // 接近速度[m/s](正で詰まりつつある)
  double need{1.45};          // 解除に要る横間隔[m]
  double floor{0.0};          // 追加解除を許す最低の現在横間隔[m]
  double max_predict_sec{1.5};  // 外挿してよい時間の上限[s]
};

struct RearEndReleaseResult
{
  bool release{false};
  bool by_prediction{false};  // 予測ぶんで解除したか(計測用)
  double sep_pred{0.0};       // 縦に並ぶ時刻での予測横間隔[m]
  double t_meet{0.0};         // 縦に並ぶまでの時間[s]
};

inline RearEndReleaseResult rearEndRelease(const RearEndReleaseInput & in)
{
  RearEndReleaseResult r;
  const double sep_now = std::abs(in.sep_now);
  r.sep_pred = sep_now;

  // 値が壊れているときは解除しない(fail-closed)。
  if (!std::isfinite(sep_now) || !std::isfinite(in.need) || in.need <= 0.0) {
    return r;
  }

  // --- 従来の条件。ここは一切変えない ---
  if (sep_now >= in.need) {
    r.release = true;
    return r;
  }

  // --- 予測ぶんの追加解除 ---
  if (!std::isfinite(in.sep_rate) || in.sep_rate <= 0.0) { return r; }   // 開いていない
  if (!std::isfinite(in.floor) || sep_now < in.floor) { return r; }      // 真後ろすぎる
  if (!std::isfinite(in.gap) || in.gap < 0.0) { return r; }
  if (!std::isfinite(in.closing) || in.closing <= 1e-3) { return r; }    // 詰まっていない

  // 縦に並ぶまでの時間。外挿しすぎない。
  const double t_meet = std::min(in.gap / in.closing, std::max(in.max_predict_sec, 0.0));
  if (!std::isfinite(t_meet) || t_meet <= 0.0) { return r; }
  r.t_meet = t_meet;
  r.sep_pred = sep_now + in.sep_rate * t_meet;
  if (r.sep_pred >= in.need) {
    r.release = true;
    r.by_prediction = true;
  }
  return r;
}

}  // namespace v2x_overtaker
