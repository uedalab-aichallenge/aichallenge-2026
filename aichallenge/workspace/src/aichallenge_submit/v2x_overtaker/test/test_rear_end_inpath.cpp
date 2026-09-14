#include "v2x_overtaker/rear_end_inpath.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using v2x_overtaker::rearEndPredictedSeparation;

namespace
{
constexpr double kRate = 1.2;     // offset_rate 既定
constexpr double kMaxT = 1.5;     // 到達時間の上限
}  // namespace

int main()
{
  // 1. 本命。右へ 2.5m 出る目標で、車間 8m・接近 3.05m/s(出せる速度差 11km/h)。
  {
    const double s = rearEndPredictedSeparation(
      -0.3, -2.5, 0.0, 8.0, 3.05, kRate, kMaxT);
    assert(s > 1.66);             // 追突防止の解除間隔を超える
    assert(std::abs(s - 2.1) < 0.05);
  }

  // 2. 相手が目前(車間2m)なら、横へ出る時間が無いので従来どおり制動側。
  {
    const double s = rearEndPredictedSeparation(
      -0.3, -2.5, 0.0, 2.0, 3.05, kRate, kMaxT);
    assert(s < 1.66);
    assert(std::abs(s - 1.087) < 0.02);   // 0.3 + 1.2*(2/3.05)
  }

  // 3. **緩めるだけではない。** 相手側へ寄せる目標なら従来より厳しくなる。
  {
    const double s = rearEndPredictedSeparation(
      2.0, 0.5, 0.0, 4.0, 3.05, kRate, kMaxT);
    const double old_min = std::min(std::abs(2.0), std::abs(0.5));
    assert(s < 2.0);
    assert(s > old_min);          // 目標そのものより手前で止まる
    assert(std::abs(s - 0.427) < 0.02);
  }

  // 4. 接近していない(closing<=0)なら判定材料が無いので従来の最小値。
  {
    const double s = rearEndPredictedSeparation(
      -0.3, -2.5, 0.0, 8.0, 0.0, kRate, kMaxT);
    assert(std::abs(s - 0.3) < 1e-9);
  }

  // 5. 車間が負(相手が後ろ)でも従来の最小値へ落とす。
  {
    const double s = rearEndPredictedSeparation(
      -0.3, -2.5, 0.0, -1.0, 3.05, kRate, kMaxT);
    assert(std::abs(s - 0.3) < 1e-9);
  }

  // 6. 壊れた値では従来の最小値(fail-closed)。
  {
    const double s = rearEndPredictedSeparation(
      -0.3, -2.5, 0.0, std::nan(""), 3.05, kRate, kMaxT);
    assert(std::abs(s - 0.3) < 1e-9);
  }

  // 7. 到達時間の上限が効く。接近が遅くても「いつかは出られる」にしない。
  {
    const double slow = rearEndPredictedSeparation(
      0.0, -5.0, 0.0, 100.0, 0.1, kRate, kMaxT);
    assert(std::abs(slow - kRate * kMaxT) < 1e-9);   // 1.8m まで
  }

  // 8. 車間が広がるほど予測間隔は単調に増える(上限まで)。
  {
    double prev = -1.0;
    for (double g = 0.5; g <= 10.0; g += 0.5) {
      const double s = rearEndPredictedSeparation(
        -0.3, -2.5, 0.0, g, 3.05, kRate, kMaxT);
      assert(s >= prev - 1e-12);
      prev = s;
    }
  }

  std::printf("test_rear_end_inpath: all passed\n");
  return 0;
}
