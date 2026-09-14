#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace v2x_overtaker
{

enum class StopNoPassRelease
{
  kNone,
  kEgoStopped,
  kTargetGoneOrMoving,
};

inline bool stopNoPassTargetRelevant(
  bool same_id, bool fresh, double gap, double look_ahead,
  double speed, double exit_speed)
{
  return same_id && fresh && std::isfinite(gap) && std::isfinite(look_ahead) &&
    std::isfinite(speed) && std::isfinite(exit_speed) && look_ahead >= 0.5 &&
    exit_speed >= 0.0 && gap >= 0.5 && gap <= look_ahead && speed <= exit_speed;
}

// Pure ownership and timeout state for the stopped-car "cannot pass" latch.
// The node decides whether the originally latched ID is still a relevant
// obstacle; this class deliberately never substitutes the current nearest
// stopped car. That prevents a changed `stopped.front()` from re-enabling
// acceleration toward the original obstacle.
class StopNoPassLatch
{
public:
  bool latched() const { return latched_; }
  const std::string & target() const { return target_; }
  double lastRelevantSec() const { return last_relevant_sec_; }

  void latch(std::string target, double now_sec)
  {
    if (target.empty()) { return; }
    latched_ = true;
    target_ = std::move(target);
    // Initializing this at latch time is essential: a target which starts
    // moving in the next callback must time out, rather than latch forever.
    last_relevant_sec_ = std::isfinite(now_sec) ? now_sec
                                                 : -std::numeric_limits<double>::infinity();
  }

  // `target_relevant` means that this exact ID is V2X-fresh, in front within
  // the stopped-car look-ahead, and no faster than the exit-speed threshold.
  // `ego_stopped` preserves the established policy of re-evaluating only once
  // the ego vehicle has stopped at a still-present stopped-car situation.
  StopNoPassRelease update(
    double now_sec, bool target_relevant, bool ego_stopped, double timeout_sec)
  {
    if (!latched_) { return StopNoPassRelease::kNone; }

    if (ego_stopped) {
      clear();
      return StopNoPassRelease::kEgoStopped;
    }

    // Invalid clock/configuration fails closed: retaining a braking latch is
    // safe, while releasing it can cause a front collision.
    if (!std::isfinite(now_sec) || !std::isfinite(timeout_sec) || timeout_sec < 0.0) {
      return StopNoPassRelease::kNone;
    }
    if (target_relevant) {
      last_relevant_sec_ = now_sec;
      return StopNoPassRelease::kNone;
    }
    if (std::isfinite(last_relevant_sec_) && now_sec - last_relevant_sec_ > timeout_sec) {
      clear();
      return StopNoPassRelease::kTargetGoneOrMoving;
    }
    return StopNoPassRelease::kNone;
  }

  // 「通れる判定に戻った」ときに外から解けるようにする。
  void clear()
  {
    latched_ = false;
    target_.clear();
    last_relevant_sec_ = -std::numeric_limits<double>::infinity();
  }

private:

  std::string target_;
  bool latched_{false};
  double last_relevant_sec_{-std::numeric_limits<double>::infinity()};
};

inline const char * stopNoPassReleaseName(StopNoPassRelease release)
{
  switch (release) {
    case StopNoPassRelease::kEgoStopped: return "自車停止";
    case StopNoPassRelease::kTargetGoneOrMoving: return "対象消失/継続走行";
    case StopNoPassRelease::kNone: break;
  }
  return "なし";
}

}  // namespace v2x_overtaker
