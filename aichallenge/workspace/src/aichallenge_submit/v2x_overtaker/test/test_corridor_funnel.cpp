#include "v2x_overtaker/corridor_funnel.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using v2x_overtaker::FunnelInput;
using v2x_overtaker::corridorFunnel;

namespace
{
// コリドア idx162-170 を模した並び(余裕 0.45m を引いた後の値)。
FunnelInput shiftingCorridor()
{
  FunnelInput in;
  in.ds = 1.4;
  in.lat_rate_mps = 1.2;
  in.speed_mps = 8.3;          // 30km/h
  const double lo[] = {-3.55, -3.60, -3.85, -3.95, -3.90, -3.75, -3.25, -2.20, -1.35};
  const double hi[] = {+0.35, +0.15, -0.05, -0.20, -0.20, +0.35, +1.40, +3.40, +2.75};
  for (int i = 0; i < 9; ++i) { in.lo.push_back(lo[i]); in.hi.push_back(hi[i]); }
  return in;
}
}  // namespace

int main()
{
  // 1. 本命。右端が先で左へ動く区間では、いまの右端を締める。
  //    コリドアそのものは -3.55 まで許すが、そこに居ると idx170 までに
  //    戻れないので壁に当たる。
  {
    const auto r = corridorFunnel(shiftingCorridor());
    assert(r.valid);
    assert(r.lo > -3.55 + 0.5);     // 明確に締まっている
    assert(r.lo < 0.0);
    assert(r.hi <= 0.35 + 1e-9);    // 左は現地点のコリドアを超えない
  }

  // 2. **一律に厳しくする変更ではない。** 低速なら横へ寄せ直せるので、
  //    漏斗はコリドアそのものに近づく。
  {
    auto in = shiftingCorridor();
    in.speed_mps = 1.5;             // 5.4km/h
    const auto r = corridorFunnel(in);
    assert(r.valid);
    assert(std::abs(r.lo - (-3.55)) < 0.05);
    assert(std::abs(r.hi - 0.35) < 0.05);
  }

  // 3. 速いほど締まる(単調)。物理的に正しい向き。
  {
    double prev = -1e9;
    for (double v = 2.0; v <= 12.0; v += 1.0) {
      auto in = shiftingCorridor();
      in.speed_mps = v;
      const auto r = corridorFunnel(in);
      assert(r.valid);
      assert(r.lo >= prev - 1e-9);  // 右端は速いほど右へ行けなくなる
      prev = r.lo;
    }
  }

  // 4. 幅が一定の区間では何も縛らない(コリドアのまま)。
  {
    FunnelInput in;
    in.ds = 1.0; in.lat_rate_mps = 1.2; in.speed_mps = 8.3;
    for (int i = 0; i < 10; ++i) { in.lo.push_back(-2.0); in.hi.push_back(2.0); }
    const auto r = corridorFunnel(in);
    assert(r.valid);
    assert(std::abs(r.lo + 2.0) < 1e-9 && std::abs(r.hi - 2.0) < 1e-9);
  }

  // 5. 先が完全に塞がっているなら、空になった地点のコリドアを返し、。
  {
    FunnelInput in;
    in.ds = 1.0; in.lat_rate_mps = 0.1; in.speed_mps = 20.0;
    const double lo[] = {-3.0, -3.0, +2.9};
    const double hi[] = {+3.0, +3.0, +3.0};
    for (int i = 0; i < 3; ++i) { in.lo.push_back(lo[i]); in.hi.push_back(hi[i]); }
    const auto r = corridorFunnel(in);
    assert(r.valid);
    assert(r.empty_at >= 0);
  }

  // 6. 壊れた入力では何も縛らない(fail-open)。ここで縛ると、
  //    データが無い区間で車が動けなくなる。
  {
    FunnelInput in;
    const auto r = corridorFunnel(in);          // 空
    assert(!r.valid);
    auto bad = shiftingCorridor();
    bad.lat_rate_mps = 0.0;
    assert(!corridorFunnel(bad).valid);
    auto nan_in = shiftingCorridor();
    nan_in.lo[3] = std::nan("");
    assert(!corridorFunnel(nan_in).valid);
  }

  // 7. 常に lo <= hi を返す(空でない限り)。
  {
    const auto r = corridorFunnel(shiftingCorridor());
    assert(r.lo <= r.hi);
  }

  std::printf("test_corridor_funnel: all passed\n");
  return 0;
}
