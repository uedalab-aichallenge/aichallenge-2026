#include "v2x_overtaker/lateral_lag.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using v2x_overtaker::lateralLagDistance;
using v2x_overtaker::prepareGateDistance;

namespace
{
constexpr double kSec = 2.7;    // 既定
constexpr double kBase = 1.0;   // 既定
constexpr double kMax = 60.0;   // 既定の上限
}  // namespace

int main()
{
  {
    const double d = lateralLagDistance(7.0, kSec, kBase, kMax);
    assert(std::abs(d - 19.9) < 0.2);
  }

  // 2. 低速では小さくなる。渋滞で「通れるのに止まる」を解く本命。
  //    5km/h = 1.39m/s
  {
    const double d = lateralLagDistance(1.39, kSec, kBase, kMax);
    assert(d < 6.0);
    assert(d > 3.0);
  }

  // 3. 高速では大きくなる(物理的に正しい向き)。35km/h = 9.72m/s
  {
    const double d = lateralLagDistance(9.72, kSec, kBase, kMax);
    assert(d > 20.0);
    assert(d < 30.0);
  }

  // 4. 停止していても base のぶんは残る(0 にはしない)。
  {
    const double d = lateralLagDistance(0.0, kSec, kBase, kMax);
    assert(std::abs(d - kBase) < 1e-9);
  }

  // 5. 上限で頭打ちになる。
  {
    const double d = lateralLagDistance(100.0, kSec, kBase, kMax);
    assert(std::abs(d - kMax) < 1e-9);
  }

  // 6. 速度が負でも 0 として扱う(後退中など)。
  {
    const double d = lateralLagDistance(-5.0, kSec, kBase, kMax);
    assert(std::abs(d - kBase) < 1e-9);
  }

  // 7. 壊れた値では安全側(最大)へ倒す。
  {
    assert(lateralLagDistance(std::nan(""), kSec, kBase, kMax) == kMax);
    assert(lateralLagDistance(7.0, std::nan(""), kBase, kMax) == kMax);
  }

  // 8. 単調性: 速いほど遅れ距離は長い。
  {
    double prev = -1.0;
    for (double v = 0.0; v <= 15.0; v += 1.0) {
      const double d = lateralLagDistance(v, kSec, kBase, kMax);
      assert(d >= prev);
      prev = d;
    }
  }

  // --- 準備を始める車間 ---

  {
    const double g = prepareGateDistance(
      9.72, 1.66, 1.2, kSec, kBase, kMax, 20.0, 1.0);
    assert(g > 35.0);
    assert(g < 50.0);
  }

  {
    const double g = prepareGateDistance(
      1.39, 1.66, 1.2, kSec, kBase, kMax, 20.0, 1.0);
    assert(std::abs(g - 20.0) < 1e-9);
  }

  // 11. 下限より小さくはならない。
  {
    const double g = prepareGateDistance(
      0.0, 0.0, 1.2, kSec, kBase, kMax, 20.0, 1.0);
    assert(g >= 20.0);
  }

  // 12. 余裕(margin)を上げると門が遠くなる。
  {
    const double g1 = prepareGateDistance(
      9.72, 1.66, 1.2, kSec, kBase, kMax, 20.0, 1.0);
    const double g2 = prepareGateDistance(
      9.72, 1.66, 1.2, kSec, kBase, kMax, 20.0, 1.5);
    assert(g2 > g1);
  }

  std::printf("test_lateral_lag: all passed\n");
  return 0;
}
