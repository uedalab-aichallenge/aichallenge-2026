#include "stuck_recovery_controller/reachable_steering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace recovery
{
Pose advanceBicycleExact(const Pose & pose, double distance, double steering, double wheel_base)
{
  const double curvature = std::tan(steering) / wheel_base;
  const double angle = distance * curvature;
  // sinc avoids cancellation for straight motion and preserves exact circular arcs.
  const double half = 0.5 * angle;
  const double sinc = std::abs(half) < 1e-8 ? 1.0 - half * half / 6.0 : std::sin(half) / half;
  return {pose.x + distance * sinc * std::cos(pose.yaw + half),
          pose.y + distance * sinc * std::sin(pose.yaw + half), pose.yaw + angle};
}

std::vector<SteeringSample> predictReachableSteering(
  const VehicleParams & vehicle, const SteeringMotion & motion,
  double command, const SteeringPredictionOptions & o)
{
  if (!std::isfinite(command) || !std::isfinite(motion.speed) ||
      !std::isfinite(motion.acceleration) || !std::isfinite(motion.steering) ||
      !std::isfinite(motion.pose.x) || !std::isfinite(motion.pose.y) ||
      !std::isfinite(motion.pose.yaw) || !std::isfinite(vehicle.wheel_base) ||
      !std::isfinite(vehicle.max_steer) || vehicle.wheel_base <= 0.0 ||
      vehicle.max_steer <= 0.0 || vehicle.max_steer >= 1.5 ||
      !std::isfinite(vehicle.front_overhang) || vehicle.front_overhang < 0.0 ||
      !std::isfinite(vehicle.rear_overhang) || vehicle.rear_overhang < 0.0 ||
      !std::isfinite(vehicle.half_width) || vehicle.half_width < 0.0 ||
      !std::isfinite(o.horizon) || o.horizon <= 0.0 || o.horizon > 5.0 ||
      !std::isfinite(o.step) || o.step < 0.001 || !std::isfinite(o.delay) || o.delay < 0.0 ||
      !std::isfinite(o.steering_rate) || o.steering_rate <= 0.0 ||
      !std::isfinite(o.spatial_step) || o.spatial_step < 0.005) { return {}; }
  command = std::clamp(command, -vehicle.max_steer, vehicle.max_steer);
  double steering = std::clamp(motion.steering, -vehicle.max_steer, vehicle.max_steer);
  double speed = motion.speed;
  double acceleration = motion.acceleration;
  Pose pose = motion.pose;
  std::vector<SteeringSample> samples{{0.0, pose, steering}};
  const double radius = std::hypot(
    std::max(vehicle.front_overhang, vehicle.rear_overhang), vehicle.half_width);
  const double speed_bound = std::abs(speed) + std::abs(acceleration) * o.horizon;
  const double body_speed = speed_bound * (1.0 + radius * std::tan(vehicle.max_steer) / vehicle.wheel_base);
  const double sample_dt = std::min(o.step, o.spatial_step / std::max(body_speed, 0.01));
  for (double time = 0.0; time < o.horizon - 1e-10;) {
    double dt = std::min(sample_dt, o.horizon - time);
    if (time < o.delay) { dt = std::min(dt, o.delay - time); }
    // Split exactly at standstill so braking never predicts motion in the opposite gear.
    if (speed * acceleration < 0.0) { dt = std::min(dt, -speed / acceleration); }
    const double next = time < o.delay ? steering :
      steering + std::clamp(command - steering, -o.steering_rate * dt, o.steering_rate * dt);
    const double distance = speed * dt + 0.5 * acceleration * dt * dt;
    pose = advanceBicycleExact(pose, distance, 0.5 * (steering + next), vehicle.wheel_base);
    const double next_speed = speed + acceleration * dt;
    if (speed * acceleration < 0.0 && std::abs(next_speed) < 1e-9) {
      speed = 0.0;
      acceleration = 0.0;
    } else { speed = next_speed; }
    steering = next;
    time += dt;
    samples.push_back({time, pose, steering});
  }
  return samples;
}

SteeringDecision selectReachableSteering(
  const VehicleParams & vehicle, const SteeringMotion & motion, double nominal,
  const SteeringPredictionOptions & o, const std::function<double(const Pose &)> & clearance)
{
  SteeringDecision result;
  result.steering = nominal;
  if (!clearance || !std::isfinite(o.clearance_margin) || o.clearance_margin < 0.0 ||
      !std::isfinite(o.escape_improvement) || o.escape_improvement <= 0.0 ||
      o.candidates < 3 || o.candidates > 201) { return result; }
  const double start = clearance(motion.pose);
  if (!std::isfinite(start)) { return result; }
  const auto evaluate = [&](double steering) {
    SteeringEvaluation e;
    const auto samples = predictReachableSteering(vehicle, motion, steering, o);
    if (samples.empty()) { return e; }
    e.minimum_clearance = start;
    for (const auto & sample : samples) {
      e.terminal_clearance = clearance(sample.pose);
      if (!std::isfinite(e.terminal_clearance)) { return e; }
      e.minimum_clearance = std::min(e.minimum_clearance, e.terminal_clearance);
    }
    e.valid = true;
    e.safe = start < 0.0
      ? e.minimum_clearance >= start - 1e-9 && e.terminal_clearance >= start + o.escape_improvement
      : e.minimum_clearance >= o.clearance_margin;
    return e;
  };
  result.nominal = evaluate(nominal);
  if (!result.nominal.valid) { return result; }
  result.valid = true;
  result.selected = result.nominal;
  if (result.nominal.safe) { return result; }
  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  double best_minimum = -std::numeric_limits<double>::infinity();
  double best_terminal = -std::numeric_limits<double>::infinity();
  const double bounded_nominal = std::clamp(nominal, -vehicle.max_steer, vehicle.max_steer);
  for (int i = -2; i < o.candidates; ++i) {
    const double candidate = i == -2 ? bounded_nominal : i == -1 ? 0.0 :
      -vehicle.max_steer + 2.0 * vehicle.max_steer * i / (o.candidates - 1);
    const auto e = evaluate(candidate);
    if (!e.valid) { continue; }
    const double distance = std::abs(candidate - bounded_nominal);
    bool take = false;
    if (e.safe) {
      take = !found || distance < best_distance;
      if (take) { found = true; best_distance = distance; }
    } else if (!found) {
      take = e.minimum_clearance > best_minimum + 1e-9 ||
        (std::abs(e.minimum_clearance - best_minimum) <= 1e-9 &&
         (e.terminal_clearance > best_terminal + 1e-9 ||
          (std::abs(e.terminal_clearance - best_terminal) <= 1e-9 && distance < best_distance)));
      if (take) {
        best_minimum = e.minimum_clearance;
        best_terminal = e.terminal_clearance;
        best_distance = distance;
      }
    }
    if (take) { result.steering = candidate; result.selected = e; }
  }
  result.overridden = result.steering != nominal;
  result.stop = !found;
  return result;
}

SteeringDecision selectReachableSteering(
  const ObstacleMap & map, const VehicleParams & vehicle, const SteeringMotion & motion,
  double nominal, const SteeringPredictionOptions & options)
{
  if (!map.valid()) { return {}; }
  return selectReachableSteering(vehicle, motion, nominal, options,
    [&](const Pose & pose) { return wallClearanceAt(map, vehicle, pose); });
}
}  // namespace recovery
