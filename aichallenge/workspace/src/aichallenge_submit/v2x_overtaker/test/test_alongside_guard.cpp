#include "v2x_overtaker/alongside_guard.hpp"

#include <cmath>

int main()
{
  using v2x_overtaker::alongsideSafeLimit;
  using v2x_overtaker::physicalPassSeparation;

// 物理的な最小間隔より小さい設定値は使わない。
  if (std::abs(physicalPassSeparation(0.80, 1.66) - 1.66) > 1e-9) { return 1; }
  if (std::abs(physicalPassSeparation(1.80, 1.66) - 1.80) > 1e-9) { return 2; }

// 現在は安全でも、相手へ寄る横目標は現在位置で止める。
  if (std::abs(alongsideSafeLimit(false, 1.66, -0.04, 1.66) - 1.66) > 1e-9) {
    return 3;
  }
// 既に近すぎるときは安全な縁まで戻す。
  if (std::abs(alongsideSafeLimit(false, 1.53, -0.14, 1.72) - 1.58) > 1e-9) {
    return 4;
  }
// 相手が左にいる場合。
  if (std::abs(alongsideSafeLimit(true, -1.66, 0.04, 1.66) + 1.66) > 1e-9) {
    return 5;
  }
// 十分離れているときは現在位置を保つ。
  if (std::abs(alongsideSafeLimit(false, 2.0, 0.0, 1.66) - 2.0) > 1e-9) {
    return 6;
  }
  return 0;
}
