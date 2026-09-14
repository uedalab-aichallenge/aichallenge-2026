#include "v2x_overtaker/lateral_interval.hpp"

#include <cmath>
#include <limits>

int main()
{
  using v2x_overtaker::resolveLateralInterval;
  using v2x_overtaker::shrinkLateralInterval;
  auto require = [](bool condition) { return condition ? 0 : 1; };

  const auto inside = resolveLateralInterval(0.4, -0.2, -1.0, 1.0);
  if (require(inside.feasible && std::abs(inside.target - 0.4) < 1e-12)) { return 1; }

  const auto clamped = resolveLateralInterval(2.0, -0.2, -1.0, 1.0);
  if (require(clamped.feasible && std::abs(clamped.target - 1.0) < 1e-12)) { return 2; }

  // Regression from d2 at epoch 1788982467.540.  The former result was the
  // midpoint 0.665, which satisfies neither [0.95,+inf] nor [-inf,0.38].
  const auto empty = resolveLateralInterval(1.4, -0.31, 0.95, 0.38);
  if (require(!empty.feasible)) { return 3; }
  if (require(std::abs(empty.target - (-0.31)) < 1e-12)) { return 4; }
  if (require(std::abs(empty.target - 0.665) > 0.5)) { return 5; }

  const auto invalid = resolveLateralInterval(
    0.0, 0.25, -1.0, std::numeric_limits<double>::quiet_NaN());
  if (require(!invalid.feasible && std::abs(invalid.target - 0.25) < 1e-12)) { return 6; }

  using v2x_overtaker::Hard;

  // 9. 下限が壁のとき、下限へ倒す。**壁を越える点を返してはならない。**。
  {
    const auto r = resolveLateralInterval(-2.5, 0.27, -0.02, -1.62, Hard::kLo);
    if (require(r.feasible)) { return 9; }
    if (require(std::abs(r.target - (-0.02)) < 1e-12)) { return 10; }
    // 壁より右(負側)へは絶対に出さない
    if (require(r.target >= -0.02 - 1e-12)) { return 11; }
  }

  {
    const auto r = resolveLateralInterval(2.5, -0.3, 1.62, 0.02, Hard::kHi);
    if (require(r.feasible)) { return 12; }
    if (require(std::abs(r.target - 0.02) < 1e-12)) { return 13; }
    if (require(r.target <= 0.02 + 1e-12)) { return 14; }
  }

  {
    const auto r = resolveLateralInterval(1.4, -0.31, 0.95, 0.38, Hard::kNone);
    if (require(!r.feasible)) { return 15; }
  }

  // 12. 交差が空でなければ、譲れない側を渡しても挙動は変わらない。
  //     既存の場面を一切変えないことの確認。
  {
    const auto a = resolveLateralInterval(0.4, -0.2, -1.0, 1.0, Hard::kLo);
    const auto b = resolveLateralInterval(0.4, -0.2, -1.0, 1.0, Hard::kNone);
    if (require(a.feasible && b.feasible)) { return 16; }
    if (require(std::abs(a.target - b.target) < 1e-12)) { return 17; }
  }

  // 13. 壊れた値では譲らない。停止のまま(安全側)。
  {
    const auto r = resolveLateralInterval(
      0.0, 0.25, -1.0, std::numeric_limits<double>::quiet_NaN(), Hard::kLo);
    if (require(!r.feasible)) { return 18; }
  }

  const auto yaw_ok = shrinkLateralInterval(-1.0, 1.0, 0.2, 0.3);
  if (require(yaw_ok.feasible && std::abs(yaw_ok.lo + 0.8) < 1e-12 &&
      std::abs(yaw_ok.hi - 0.7) < 1e-12)) { return 19; }

  // A pose wider than the base wall band cannot be repaired while stationary.
  // The caller must retain the base band and command a slow centering motion.
  const auto yaw_empty = shrinkLateralInterval(-0.24, 0.75, 0.8, 0.7);
  if (require(!yaw_empty.feasible && yaw_empty.lo > yaw_empty.hi)) { return 20; }
  return 0;
}
