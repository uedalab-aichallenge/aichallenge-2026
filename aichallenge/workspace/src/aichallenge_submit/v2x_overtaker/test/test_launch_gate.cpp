#include "v2x_overtaker/launch_gate.hpp"

int main()
{
  using v2x_overtaker::launchMotionObserved;
  using v2x_overtaker::launchWindowActive;
  using v2x_overtaker::suppressStoppedCarsAtLaunch;

  if (launchMotionObserved(0.0, 0.0, 0.3)) { return 1; }
  if (!launchMotionObserved(0.0, 0.31, 0.3)) { return 2; }
  if (!launchMotionObserved(-0.31, 0.0, 0.3)) { return 3; }

// 発進前と発進後 duration 秒以内だけ発進区間になる。
  if (!launchWindowActive(-1.0, 100.0, 6.0)) { return 4; }
  if (!launchWindowActive(100.0, 105.9, 6.0)) { return 5; }
  if (launchWindowActive(100.0, 106.0, 6.0)) { return 6; }

  if (!suppressStoppedCarsAtLaunch(-1.0, 100.0, 0.6)) { return 7; }
  if (!suppressStoppedCarsAtLaunch(100.0, 100.5, 0.6)) { return 8; }
  if (suppressStoppedCarsAtLaunch(100.0, 100.7, 0.6)) { return 9; }
  return 0;
}
