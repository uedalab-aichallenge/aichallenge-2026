#include "v2x_overtaker/stop_nopass_latch.hpp"

#include <cstdlib>
#include <iostream>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
}  // namespace

int main()
{
  using v2x_overtaker::StopNoPassLatch;
  using v2x_overtaker::StopNoPassRelease;
  using v2x_overtaker::stopNoPassTargetRelevant;

  // Entry classification may reject 1.1m/s as moving, but the same latched ID
  // remains an avoidance obstacle until it exceeds the 1.5m/s exit threshold.
  require(stopNoPassTargetRelevant(true, true, 12.0, 40.0, 1.1, 1.5),
    "same-ID threshold jitter must remain in the avoidance calculation");
  require(!stopNoPassTargetRelevant(true, true, 12.0, 40.0, 1.6, 1.5),
    "target above the exit threshold must begin its release grace");
  require(!stopNoPassTargetRelevant(false, true, 12.0, 40.0, 0.0, 1.5),
    "a different ID must never inherit the latch");

  // 1: raw speed may cross the stopped-entry threshold (1.0 m/s) without
  // releasing. The caller's exit-speed hysteresis classifies these samples as
  // relevant, so a same-ID 0.9/1.1m/s oscillation refreshes the timer.
  StopNoPassLatch jitter;
  jitter.latch("d2", 0.0);
  require(jitter.update(0.40, true, false, 1.0) == StopNoPassRelease::kNone,
    "same-ID 0.9/1.1 speed jitter must retain the latch");
  require(jitter.update(1.20, true, false, 1.0) == StopNoPassRelease::kNone &&
    jitter.latched() && jitter.target() == "d2",
    "fresh same-ID observation must refresh the timeout");

  // 2: another stopped car becoming nearest is not a release event; the node
  // still supplies true because the original d2 remains relevant.
  StopNoPassLatch front_swap;
  front_swap.latch("d2", 0.0);
  require(front_swap.update(0.80, true, false, 1.0) == StopNoPassRelease::kNone &&
    front_swap.latched() && front_swap.target() == "d2",
    "stopped.front replacement must not release the original target");

  // 3: sustained exit-speed movement releases only after the one-second
  // grace, never at the first fast V2X sample.
  StopNoPassLatch moving;
  moving.latch("d2", 0.0);
  require(moving.update(1.00, false, false, 1.0) == StopNoPassRelease::kNone,
    "one-second boundary must remain conservative");
  require(moving.update(1.01, false, false, 1.0) == StopNoPassRelease::kTargetGoneOrMoving &&
    !moving.latched() && moving.target().empty(),
    "sustained exit-speed movement must release and reset ownership");

  // 4: a passed, stale, or otherwise no-longer-forward target uses the same
  // timeout and cannot leave a permanent latch.
  StopNoPassLatch gone;
  gone.latch("d2", 4.0);
  require(gone.update(5.01, false, false, 1.0) == StopNoPassRelease::kTargetGoneOrMoving &&
    !gone.latched(), "passed/stale target must release after the timeout");

  // 5: retain the legacy policy: after the ego vehicle has stopped in the
  // stopped-car branch, clear once so geometry can be evaluated again.
  StopNoPassLatch ego_stop;
  ego_stop.latch("d2", 0.0);
  require(ego_stop.update(0.05, true, true, 1.0) == StopNoPassRelease::kEgoStopped &&
    !ego_stop.latched() && ego_stop.target().empty(),
    "ego-stop re-evaluation must clear and reset the latch");

  return 0;
}
