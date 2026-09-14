#ifndef STUCK_RECOVERY_CONTROLLER__REACHABLE_STEERING_HPP_
#define STUCK_RECOVERY_CONTROLLER__REACHABLE_STEERING_HPP_

#include "stuck_recovery_controller/recovery_planner.hpp"

#include <functional>
#include <vector>

namespace recovery
{
struct SteeringPredictionOptions
{
  double horizon{0.8};          // s
  double step{0.02};            // s
  double delay{0.20};           // s before a new steering command acts
  double steering_rate{1.05};   // physical rad/s
  double spatial_step{0.05};    // m maximum body-point travel between samples
  double clearance_margin{0.0}; // m
  double escape_improvement{0.01}; // m
  int candidates{41};
};

struct SteeringMotion
{
  Pose pose;
  double speed{0.0};            // signed m/s
  double acceleration{0.0};     // signed m/s^2, brake acceleration opposes speed
  double steering{0.0};         // measured physical rad
};

struct SteeringSample
{
  double time;
  Pose pose;
  double steering;
};

struct SteeringEvaluation
{
  bool valid{false};
  bool safe{false};
  double minimum_clearance{0.0};
  double terminal_clearance{0.0};
};

struct SteeringDecision
{
  bool valid{false};
  bool overridden{false};
  bool stop{false};
  double steering{0.0};
  SteeringEvaluation nominal;
  SteeringEvaluation selected;
};

Pose advanceBicycleExact(const Pose & pose, double distance, double steering, double wheel_base);
std::vector<SteeringSample> predictReachableSteering(
  const VehicleParams & vehicle, const SteeringMotion & motion,
  double commanded_steering, const SteeringPredictionOptions & options);

// The caller supplies a clearance function; the map overload measures the whole body.
SteeringDecision selectReachableSteering(
  const VehicleParams & vehicle, const SteeringMotion & motion, double nominal_steering,
  const SteeringPredictionOptions & options, const std::function<double(const Pose &)> & clearance);
SteeringDecision selectReachableSteering(
  const ObstacleMap & map, const VehicleParams & vehicle, const SteeringMotion & motion,
  double nominal_steering, const SteeringPredictionOptions & options);
}  // namespace recovery

#endif
