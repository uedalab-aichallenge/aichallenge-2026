#ifndef V2X_OVERTAKER__LATERAL_INTERVAL_HPP_
#define V2X_OVERTAKER__LATERAL_INTERVAL_HPP_

#include <algorithm>
#include <cmath>

namespace v2x_overtaker
{

struct LateralIntervalDecision
{
  double target;
  bool feasible;
};

struct ShrunkLateralInterval
{
  double lo;
  double hi;
  bool feasible;
};

inline ShrunkLateralInterval shrinkLateralInterval(
  double lo, double hi, double add_lo, double add_hi)
{
  const bool finite = std::isfinite(lo) && std::isfinite(hi) &&
    std::isfinite(add_lo) && std::isfinite(add_hi);
  if (!finite || lo > hi || add_lo < 0.0 || add_hi < 0.0) {
    return {lo, hi, false};
  }
  const double shrunk_lo = lo + add_lo;
  const double shrunk_hi = hi - add_hi;
  return {shrunk_lo, shrunk_hi, shrunk_lo <= shrunk_hi};
}

// 交差が空のとき、どちらの制約を立てるか。
enum class Hard { kNone, kLo, kHi, kNeither };

inline LateralIntervalDecision resolveLateralInterval(
  double intent, double fallback, double lo, double hi,
  Hard hard = Hard::kNone)
{
  const bool finite = std::isfinite(intent) && std::isfinite(fallback) &&
    std::isfinite(lo) && std::isfinite(hi);
  if (!finite) {
    return {std::isfinite(fallback) ? fallback : 0.0, false};
  }
  if (lo > hi) {
    // 譲れない側が分かっているときだけ、そちらを立てて交差を解く。
    // 返す点は必ず壁の内側になる(壁そのものの境界値)。
    if (hard == Hard::kLo) { return {lo, true}; }
    if (hard == Hard::kHi) { return {hi, true}; }
    // どちらも譲れる制約なら、両者の中点(=各相手からの余裕の最小値が
    // 最大になる点)へ。止まっても矛盾は解けないので縦は止めない。
    if (hard == Hard::kNeither) { return {0.5 * (lo + hi), true}; }
    return {std::isfinite(fallback) ? fallback : 0.0, false};
  }
  return {std::clamp(intent, lo, hi), true};
}

}  // namespace v2x_overtaker

#endif  // V2X_OVERTAKER__LATERAL_INTERVAL_HPP_
