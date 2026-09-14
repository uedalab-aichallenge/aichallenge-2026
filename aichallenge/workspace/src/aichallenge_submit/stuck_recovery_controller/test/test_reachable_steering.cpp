#include "stuck_recovery_controller/reachable_steering.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
bool near(double a, double b) { return std::abs(a - b) < 1e-9; }
}

int main()
{
  recovery::VehicleParams vehicle;
  vehicle.wheel_base = 1.0;
  vehicle.max_steer = 0.3;
  recovery::SteeringPredictionOptions options;
  options.candidates = 13;
  options.steering_rate = 1.0;
  recovery::SteeringMotion motion;
  motion.speed = 2.0;
  motion.steering = 0.2;

  const auto reversal = recovery::predictReachableSteering(vehicle, motion, -0.3, options);
  require(!reversal.empty(), "prediction must produce samples");
  bool delayed_turn = false;
  for (std::size_t i = 1; i < reversal.size(); ++i) {
    const auto & a = reversal[i - 1];
    const auto & b = reversal[i];
    require(std::abs(b.steering - a.steering) <= options.steering_rate * (b.time - a.time) + 1e-9,
      "steering must respect physical rate bound");
    if (b.time <= options.delay + 1e-9) {
      require(near(b.steering, motion.steering), "reversal cannot act during transport delay");
      delayed_turn = b.pose.yaw > 0.0;
    }
  }
  require(delayed_turn, "vehicle keeps turning toward previous steering during reversal delay");
  require(reversal.back().steering < 0.0, "reversal must eventually take effect");

  const recovery::Pose origin;
  const double steer = std::atan(0.5);
  const auto forward = recovery::advanceBicycleExact(origin, 2.0, steer, 1.0);
  require(near(forward.x, 2.0 * std::sin(1.0)) && near(forward.y, 2.0 * (1.0 - std::cos(1.0))),
    "constant-curvature integration must follow the exact circle");
  const auto returned = recovery::advanceBicycleExact(forward, -2.0, steer, 1.0);
  require(near(returned.x, 0.0) && near(returned.y, 0.0) && near(returned.yaw, 0.0),
    "reverse distance must invert forward bicycle motion");
  motion.steering = 0.1;
  const auto positive = recovery::predictReachableSteering(vehicle, motion, 0.1, options);
  motion.speed = -2.0;
  const auto negative = recovery::predictReachableSteering(vehicle, motion, 0.1, options);
  require(positive.back().pose.yaw > 0.0 && negative.back().pose.yaw < 0.0 && negative.back().pose.x < 0.0,
    "reverse gear must reverse yaw rate as well as longitudinal motion");

  motion.speed = 2.0;
  motion.steering = 0.0;
  const double nominal = 0.123456789;
  const auto unchanged = recovery::selectReachableSteering(vehicle, motion, nominal, options,
    [](const recovery::Pose &) { return 2.0; });
  require(unchanged.valid && !unchanged.stop && !unchanged.overridden &&
    std::memcmp(&nominal, &unchanged.steering, sizeof(double)) == 0,
    "safe nominal steering must remain bitwise unchanged");

  const auto closest = recovery::selectReachableSteering(vehicle, motion, 0.3, options,
    [](const recovery::Pose & p) { return 0.065 - p.yaw; });
  require(closest.valid && closest.overridden && !closest.stop && near(closest.steering, 0.05),
    "unsafe nominal must select the closest safe sampled steering");
  require(closest.selected.minimum_clearance >= 0.0, "selected swept motion must be safe");

  const auto blocked = recovery::selectReachableSteering(vehicle, motion, -0.3, options,
    [](const recovery::Pose & p) { return 0.1 - p.x + 0.1 * p.y; });
  require(blocked.valid && blocked.stop && near(blocked.steering, 0.3),
    "all-unsafe motion must brake and select maximum minimum clearance");

  const auto escape = recovery::selectReachableSteering(vehicle, motion, 0.0, options,
    [](const recovery::Pose & p) { return -0.1 + p.x; });
  require(escape.valid && !escape.stop && !escape.overridden,
    "non-worsening motion improving initial wall penetration must remain available");
  motion.speed = -2.0;
  const auto worsens = recovery::selectReachableSteering(vehicle, motion, 0.0, options,
    [](const recovery::Pose & p) { return -0.1 + p.x; });
  require(worsens.stop, "initial wall penetration must never permit deeper penetration");

  motion.speed = 0.2;
  motion.acceleration = -1.0;
  const auto braking = recovery::predictReachableSteering(vehicle, motion, 0.0, options);
  require(near(braking.back().pose.x, 0.02), "braking must stop without reversing simulated direction");
  motion.acceleration = std::numeric_limits<double>::quiet_NaN();
  require(recovery::predictReachableSteering(vehicle, motion, 0.0, options).empty(),
    "nonfinite dynamics must fail validation");
  require(!recovery::selectReachableSteering(recovery::ObstacleMap{}, vehicle, motion, 0.0, options).valid,
    "missing map must not claim a certified prediction");
  std::cout << "reachable steering invariants passed\n";
}
