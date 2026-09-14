#include "stuck_recovery_controller/escape_certificate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace recovery
{
namespace
{

bool validPhaseRanges(const Plan & plan)
{
  if (plan.phases.empty() || plan.path.empty()) { return false; }
  for (std::size_t i = 0; i < plan.phases.size(); ++i) {
    const auto & phase = plan.phases[i];
    if (phase.path_begin >= phase.path_end || phase.path_end > plan.path.size()) {
      return false;
    }
    if (i == 0 && phase.path_begin != 0) { return false; }
    if (i > 0 && phase.path_begin + 1 != plan.phases[i - 1].path_end) {
      return false;
    }
  }
  return plan.phases.back().path_end == plan.path.size();
}

bool finiteClearances(const std::vector<double> & clearances)
{
  return std::all_of(clearances.begin(), clearances.end(), [](double value) {
    return std::isfinite(value);
  });
}

bool certifyClearance(
  double start, const std::vector<double> & path, const EscapeCertificateOptions & options,
  const char * label, std::string & reason)
{
  const double minimum = *std::min_element(path.begin(), path.end());
  if (start >= 0.0) {
    if (minimum < 0.0) {
      reason = std::string(label) + " path enters a new overlap";
      return false;
    }
  } else if (minimum < start - options.plan_worsen_tolerance) {
    // 終端が確実に外へ出るなら、抜けるまでの一時的な悪化を許す(round116)。
    const bool escapes = path.back() >= options.terminal_clearance;
    if (!escapes || minimum < start - options.plan_escape_dip) {
      reason = std::string(label) + " overlap worsens below its starting clearance";
      return false;
    }
  }
  if (path.back() < options.terminal_clearance) {
    reason = std::string(label) + " terminal clearance remains negative";
    return false;
  }
  return true;
}

}  // namespace

double EscapeCertificate::runtimeWallFloor(std::size_t path_index) const
{
  if (!valid || planned_wall_clearance.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  // 実行時の下限は「証明した計画が**これから**沈む深さ」を基準にする。
  double ahead = minimum_wall_clearance;
  if (path_index < planned_wall_clearance.size()) {
    ahead = *std::min_element(
      planned_wall_clearance.begin() + static_cast<std::ptrdiff_t>(path_index),
      planned_wall_clearance.end());
  }
  const double planned_floor = std::min(start_wall_clearance, ahead);
  return starts_inside_wall
    ? planned_floor - options.runtime_start_worsen_tolerance
    : -options.runtime_start_worsen_tolerance;
}

bool EscapeCertificate::allowsRuntimeWallClearance(
  std::size_t path_index, double measured) const
{
  return std::isfinite(measured) && measured >= runtimeWallFloor(path_index);
}

bool EscapeCertificate::matchesPlan(const Plan & plan) const
{
  if (!valid || !plan.valid || plan.path.size() != planned_path.size() ||
      plan.phases.size() != planned_phases.size())
  {
    return false;
  }
  for (std::size_t i = 0; i < plan.path.size(); ++i) {
    if (plan.path[i].x != planned_path[i].x || plan.path[i].y != planned_path[i].y ||
        plan.path[i].yaw != planned_path[i].yaw)
    {
      return false;
    }
  }
  for (std::size_t i = 0; i < plan.phases.size(); ++i) {
    const auto & actual = plan.phases[i];
    const auto & accepted = planned_phases[i];
    if (actual.forward != accepted.forward || actual.steer != accepted.steer ||
        actual.length != accepted.length || actual.path_begin != accepted.path_begin ||
        actual.path_end != accepted.path_end)
    {
      return false;
    }
  }
  return true;
}

EscapeCertificate certifyEscapePlan(
  const Plan & plan,
  double start_wall_clearance,
  const std::vector<double> & wall_clearances,
  double start_car_clearance,
  const std::vector<double> & car_clearances,
  const EscapeCertificateOptions & options,
  const std::vector<CarClearanceTrace> & car_traces)
{
  EscapeCertificate certificate;
  certificate.options = options;
  certificate.start_wall_clearance = start_wall_clearance;
  certificate.start_car_clearance = start_car_clearance;
  certificate.starts_inside_wall = start_wall_clearance < 0.0;
  certificate.starts_overlapping_car = start_car_clearance < 0.0;

  if (!plan.valid) {
    certificate.reason = "planner returned an invalid plan";
    return certificate;
  }
  if (!validPhaseRanges(plan)) {
    certificate.reason = "phase ranges do not cover one immutable whole path";
    return certificate;
  }
  if (wall_clearances.size() != plan.path.size() ||
      car_clearances.size() != plan.path.size())
  {
    certificate.reason = "clearance samples do not match plan path";
    return certificate;
  }
  if (!std::isfinite(start_wall_clearance) || !std::isfinite(start_car_clearance) ||
      !finiteClearances(wall_clearances) || !finiteClearances(car_clearances))
  {
    certificate.reason = "clearance contains a non-finite value";
    return certificate;
  }
  if (std::abs(wall_clearances.front() - start_wall_clearance) >
        options.plan_worsen_tolerance ||
      std::abs(car_clearances.front() - start_car_clearance) >
        options.plan_worsen_tolerance)
  {
    certificate.reason = "path origin does not match the measured start";
    return certificate;
  }
  if (options.plan_worsen_tolerance < 0.0 || options.terminal_clearance < 0.0 ||
      options.terminal_car_progress <= 0.0 ||
      options.runtime_start_worsen_tolerance < 0.0 ||
      options.runtime_progress_worsen_tolerance < 0.0 ||
      options.runtime_car_progress_worsen_tolerance < 0.0)
  {
    certificate.reason = "certificate tolerance is invalid";
    return certificate;
  }

  certificate.minimum_wall_clearance =
    *std::min_element(wall_clearances.begin(), wall_clearances.end());
  certificate.terminal_wall_clearance = wall_clearances.back();
  certificate.minimum_car_clearance =
    *std::min_element(car_clearances.begin(), car_clearances.end());
  certificate.terminal_car_clearance = car_clearances.back();

  if (!certifyClearance(
      start_wall_clearance, wall_clearances, options, "wall", certificate.reason))
  {
    return certificate;
  }
  if (car_traces.empty()) {
    if (!certifyClearance(
        start_car_clearance, car_clearances, options, "car", certificate.reason))
    {
      return certificate;
    }
  } else {
    std::unordered_set<std::string> ids;
    double start_penetration = 0.0;
    double terminal_penetration = 0.0;
    for (const auto & trace : car_traces) {
      if (trace.vehicle_id.empty() || !ids.insert(trace.vehicle_id).second) {
        certificate.reason = "per-car clearance has an empty or duplicate vehicle id";
        return certificate;
      }
      if (trace.path_clearance.size() != plan.path.size() ||
          !std::isfinite(trace.start_clearance) ||
          !finiteClearances(trace.path_clearance))
      {
        certificate.reason = "per-car clearance samples are invalid";
        return certificate;
      }
      if (std::abs(trace.path_clearance.front() - trace.start_clearance) >
          options.plan_worsen_tolerance)
      {
        certificate.reason = "per-car path origin does not match the measured start";
        return certificate;
      }
      const double minimum =
        *std::min_element(trace.path_clearance.begin(), trace.path_clearance.end());
      const double terminal = trace.path_clearance.back();
      if (trace.start_clearance >= 0.0) {
        if (minimum < 0.0) {
          certificate.reason = "per-car path enters a new overlap with " + trace.vehicle_id;
          return certificate;
        }
      } else {
        if (minimum < trace.start_clearance - options.plan_worsen_tolerance) {
          certificate.reason = "per-car overlap worsens with " + trace.vehicle_id;
          return certificate;
        }
        start_penetration += -trace.start_clearance;
        terminal_penetration += std::max(0.0, -terminal);
      }
      certificate.certified_cars.push_back(CertifiedCarTrace{
        trace.vehicle_id, trace.start_clearance < 0.0, trace.start_clearance,
        minimum, terminal, trace.path_clearance});
    }
    if (start_penetration > 0.0 && terminal_penetration > 0.0 &&
        terminal_penetration > start_penetration - options.terminal_car_progress)
    {
      certificate.reason = "per-car terminal overlap has no material total progress";
      certificate.certified_cars.clear();
      return certificate;
    }
  }

  certificate.planned_wall_clearance = wall_clearances;
  certificate.planned_car_clearance = car_clearances;
  certificate.planned_path = plan.path;
  certificate.planned_phases = plan.phases;
  certificate.valid = true;
  certificate.reason = "certified";
  return certificate;
}

}  // namespace recovery
