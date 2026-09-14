#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace v2x_overtaker
{

// 「先で入れない場所へ通じる横位置には、いま行かない」を計算する。

struct FunnelInput
{
  // 自車の直前から先へ向かって並べた区間。lo/hi は既に余裕を引いた可動域。
  std::vector<double> lo;
  std::vector<double> hi;
  double ds{1.0};             // 区間ごとの進行距離[m]
  double lat_rate_mps{1.2};   // 横移動レート[m/s]
  double speed_mps{8.3};      // 進行速度[m/s]
  double speed_floor{1.0};    // 低速での発散を防ぐ下限[m/s]
};

struct FunnelResult
{
  double lo{-1e9};
  double hi{1e9};
  bool valid{false};
  // 漏斗が空になった地点(手前から数えた添字)。-1 なら空にならなかった。
  int empty_at{-1};
};

// 先端から手前へ戻して、いまの地点で許される横位置の範囲を返す。
//
// 途中で空になったら、そこまでの結果を返す(valid=true, empty_at>=0)。
// 空になるのは「その先を通り抜ける横位置が存在しない」場合で、
// これ自体は情報なので握り潰さない。呼び出し側は手前側の結果を使えばよい。
inline FunnelResult corridorFunnel(const FunnelInput & in)
{
  FunnelResult r;
  const std::size_t n = std::min(in.lo.size(), in.hi.size());
  if (n == 0 || !std::isfinite(in.ds) || in.ds <= 0.0 ||
      !std::isfinite(in.lat_rate_mps) || in.lat_rate_mps <= 0.0 ||
      !std::isfinite(in.speed_mps) || !std::isfinite(in.speed_floor))
  {
    return r;                                   // 壊れた入力では何も縛らない
  }
  const double v = std::max(in.speed_mps, std::max(in.speed_floor, 0.1));
  // 進行 1m あたりに動ける横幅。速いほど小さい。
  const double per_m = in.lat_rate_mps / v;
  const double grow = per_m * in.ds;

  std::size_t last = n - 1;
  if (!std::isfinite(in.lo[last]) || !std::isfinite(in.hi[last]) ||
      in.lo[last] > in.hi[last])
  {
    return r;
  }
  double L = in.lo[last];
  double H = in.hi[last];
  for (std::size_t k = last; k-- > 0; ) {
    if (!std::isfinite(in.lo[k]) || !std::isfinite(in.hi[k])) { return r; }
    L = std::max(in.lo[k], L - grow);
    H = std::min(in.hi[k], H + grow);
    if (L > H) {
      // ここから先は通り抜けられない。手前の結果は使えないので、。
      r.lo = in.lo[k];
      r.hi = in.hi[k];
      r.valid = true;
      r.empty_at = static_cast<int>(k);
      return r;
    }
  }
  r.lo = L;
  r.hi = H;
  r.valid = true;
  return r;
}

}  // namespace v2x_overtaker
