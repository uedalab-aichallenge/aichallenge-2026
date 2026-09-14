#include "stuck_recovery_controller/escape_certificate.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

namespace
{
void require(bool ok, const char * message)
{
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}

recovery::Plan twoPhasePlan()
{
  recovery::Plan plan;
  plan.valid = true;
  plan.path = {{0, 0, 0}, {-1, 0, 0}, {-2, 0, 0}, {-1, 0, 0}, {0, 0, 0}};
  plan.phases = {{false, 0.0, 2.0, 0, 3}, {true, 0.0, 2.0, 2, 5}};
  return plan;
}
}  // namespace

int main()
{
  const auto plan = twoPhasePlan();
  const std::vector<double> safe_wall{-0.94, -0.82, -0.71, -0.58, 0.50};
  const std::vector<double> safe_car{2.0, 2.1, 2.2, 2.3, 2.4};
  const auto safe = recovery::certifyEscapePlan(plan, -0.94, safe_wall, 2.0, safe_car);
  require(safe.valid, "a non-worsening whole escape must be certified");
  require(safe.matchesPlan(plan), "the certificate must identify the exact accepted plan");
  require(std::abs(safe.runtimeWallFloor(2) - (-0.99)) < 1e-9,
    "runtime floor must enforce the measured starting overlap");
  require(safe.allowsRuntimeWallClearance(2, -0.98), "in-envelope motion must be allowed");
  require(!safe.allowsRuntimeWallClearance(2, -1.00), "deeper overlap must abort");

  auto changed = plan;
  changed.path.back().x += 0.01;
  require(!safe.matchesPlan(changed), "a changed path must invalidate its certificate");
  changed = plan;
  changed.phases.front().path_end -= 1;
  require(!safe.matchesPlan(changed), "changed phase ranges must invalidate the certificate");

  auto worsens = safe_wall;
  worsens[1] = -1.00;
  require(!recovery::certifyEscapePlan(plan, -0.94, worsens, 2.0, safe_car).valid,
    "an escape that deepens the initial wall overlap must be rejected");
  auto trapped = safe_wall;
  trapped.back() = -0.01;
  require(!recovery::certifyEscapePlan(plan, -0.94, trapped, 2.0, safe_car).valid,
    "an escape that ends inside the wall must be rejected");
  auto enters_wall = safe_wall;
  enters_wall.front() = 0.10;
  require(!recovery::certifyEscapePlan(plan, 0.10, enters_wall, 2.0, safe_car).valid,
    "a plan starting outside must not enter a wall overlap");
  auto hits_car = safe_car;
  hits_car[3] = -0.10;
  require(!recovery::certifyEscapePlan(plan, -0.94, safe_wall, 2.0, hits_car).valid,
    "a plan starting clear must not enter another car");
  const auto trace = [](const char * id, double start, std::vector<double> path) {
    return recovery::CarClearanceTrace{id, start, std::move(path)};
  };
  const std::vector<double> aggregate_improves{-1.0, -0.9, -0.8, -0.7, -0.6};
  auto per_car = std::vector<recovery::CarClearanceTrace>{
    trace("A", -1.0, {-1.0, -0.95, -0.90, -0.85, -0.80}),
    trace("B", 0.10, {0.10, 0.08, 0.03, 0.00, -0.05})};
  require(!recovery::certifyEscapePlan(
      plan, -0.94, safe_wall, -1.0, aggregate_improves, {}, per_car).valid,
    "aggregate improvement must not hide a new overlap with another vehicle");
  per_car = {
    trace("A", -1.0, {-1.0, -0.90, -0.80, -0.70, -0.60}),
    trace("B", -0.20, {-0.20, -0.21, -0.22, -0.23, -0.24})};
  require(!recovery::certifyEscapePlan(
      plan, -0.94, safe_wall, -1.0, aggregate_improves, {}, per_car).valid,
    "total progress must not hide worsening with an already overlapping vehicle");
  per_car = {
    trace("A", -1.0, {-1.0, -0.99, -0.95, -0.92, -0.89}),
    trace("B", -0.20, {-0.20, -0.20, -0.20, -0.20, -0.20})};
  require(recovery::certifyEscapePlan(
      plan, -0.94, safe_wall, -1.0, aggregate_improves, {}, per_car).valid,
    "per-ID non-worsening with 0.10m total progress must certify a partial escape");
  per_car[0].path_clearance.back() = -0.91;
  require(!recovery::certifyEscapePlan(
      plan, -0.94, safe_wall, -1.0, aggregate_improves, {}, per_car).valid,
    "less than 0.10m total overlap progress must be rejected");
  per_car = {
    trace("A", -0.10, {-0.10, -0.08, -0.05, -0.02, 0.01}),
    trace("B", -0.05, {-0.05, -0.04, -0.03, -0.01, 0.01})};
  require(recovery::certifyEscapePlan(
      plan, -0.94, safe_wall, -0.10, {-0.10, -0.08, -0.05, -0.02, 0.01}, {}, per_car).valid,
    "full per-ID separation must certify regardless of the progress threshold");
  auto duplicate = per_car;
  duplicate[1].vehicle_id = "A";
  require(!recovery::certifyEscapePlan(
      plan, -0.94, safe_wall, -0.10, {-0.10, -0.08, -0.05, -0.02, 0.01}, {}, duplicate).valid,
    "duplicate vehicle IDs must fail closed");
  auto empty_id = per_car;
  empty_id[0].vehicle_id.clear();
  require(!recovery::certifyEscapePlan(
      plan, -0.94, safe_wall, -0.10, {-0.10, -0.08, -0.05, -0.02, 0.01}, {}, empty_id).valid,
    "empty vehicle IDs must fail closed");
  auto wrong_origin = safe_wall;
  wrong_origin.front() = -0.80;
  require(!recovery::certifyEscapePlan(plan, -0.94, wrong_origin, 2.0, safe_car).valid,
    "the sampled path origin must match the measured start");
  auto invalid_ranges = plan;
  invalid_ranges.phases[1].path_begin = 1;
  require(!recovery::certifyEscapePlan(
      invalid_ranges, -0.94, safe_wall, 2.0, safe_car).valid,
    "phase ranges must cover one immutable whole path");
  auto nonfinite = safe_wall;
  nonfinite[0] = std::numeric_limits<double>::quiet_NaN();
  require(!recovery::certifyEscapePlan(plan, -0.94, nonfinite, 2.0, safe_car).valid,
    "non-finite geometry must be rejected");

  std::cout << "escape certificate invariants passed\n";
}
