#include "stuck_recovery_controller/phase_path.hpp"
#include "stuck_recovery_controller/escape_certificate.hpp"

#include <cstdlib>
#include <iostream>

namespace
{
void require(bool ok, const char * message)
{
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}

void checkPlan(const recovery::Plan & plan)
{
  require(plan.valid && !plan.phases.empty(), "planner must produce a plan");
  for (std::size_t p = 0; p < plan.phases.size(); ++p) {
    const auto & phase = plan.phases[p];
    require(phase.path_begin < phase.path_end && phase.path_end <= plan.path.size(),
      "planner phase range must be valid");
    if (p > 0) {
      require(phase.path_begin + 1 == plan.phases[p - 1].path_end,
        "adjacent phases must share the switching point");
    }
    for (std::size_t i = phase.path_begin + 1; i < phase.path_end; ++i) {
      const auto & a = plan.path[i - 1];
      const auto & b = plan.path[i];
      const double longitudinal = (b.x - a.x) * std::cos(a.yaw) +
        (b.y - a.y) * std::sin(a.yaw);
      require(phase.forward ? longitudinal > 0.0 : longitudinal < 0.0,
        "path segments must agree with phase direction");
    }
  }
  require(plan.phases.back().path_end == plan.path.size(), "last phase must reach path end");
}
}  // namespace

int main()
{
  // Only travel in the requested direction advances a phase.  This is the
  // regression for a 0.5m reverse phase that used to complete on forward
  // inertia because the controller accumulated hypot(dx, dy).
  const recovery::Pose origin{0.0, 0.0, 0.0};
  const recovery::Pose forward_one{1.0, 0.0, 0.0};
  require(recovery::directedTravelIncrement(origin, forward_one, true) > 0.99,
    "forward motion must advance a forward phase");
  require(recovery::directedTravelIncrement(origin, forward_one, false) < -0.99,
    "forward inertia must move reverse progress backwards");
  const recovery::Pose reverse_one{-1.0, 0.0, 0.0};
  require(recovery::directedTravelIncrement(origin, reverse_one, false) > 0.99,
    "reverse motion must advance a reverse phase");
  require(recovery::directedTravelIncrement(origin, reverse_one, true) < -0.99,
    "reverse inertia must move forward progress backwards");
  const recovery::Pose quarter_turn{2.0 / M_PI, 2.0 / M_PI, M_PI_2};
  require(recovery::directedTravelIncrement(origin, quarter_turn, true) > 0.89,
    "mid-heading projection must measure curved forward motion");

  // A reverse arc already contains the signed yaw change in increasing index order.
  // Its synthetic lookahead tail must continue that turn, not bend the other way.
  const recovery::Pose arc_prev{0.0, 0.0, 0.0};
  const recovery::Pose arc_last{-0.25, 0.0, -0.10};
  const auto curved_tail = recovery::makeReverseTail(arc_last, arc_prev, 0.5, 0.25);
  require(curved_tail.size() == 2, "reverse tail must have the requested samples");
  require(curved_tail.front().yaw < arc_last.yaw &&
    curved_tail.back().yaw < curved_tail.front().yaw,
    "reverse tail yaw must continue the signed curvature");
  const auto straight_tail = recovery::makeReverseTail(arc_last, std::nullopt, 0.5, 0.25);
  require(straight_tail.size() == 2 && straight_tail.back().yaw == arc_last.yaw,
    "one-point reverse path must receive a straight tail");

  // Four overlapping branches: a later forward point is closer than the
  // remaining reverse points. A global monotonic nearest search jumps to it.
  recovery::Plan plan;
  for (double x : {0.0, -1.0, -2.0, -1.0, 0.0, -1.0, -2.0, -1.0, 0.0}) {
    plan.path.push_back({x, 0.0, 0.0});
  }
  plan.phases = {{false, 0, 2, 0, 3}, {true, 0, 2, 2, 5},
    {false, 0, 2, 4, 7}, {true, 0, 2, 6, 9}};
  plan.rev_points = 3;
  plan.valid = true;
  checkPlan(plan);
  const recovery::Pose current{0, 0, 0};
  require(recovery::nearestInPhase(plan, 0, 1, current) == 1,
    "reverse must not jump to the closer forward branch");
  require(recovery::nearestInPhase(plan, 1, 1, {-1, 0, 0}) == 3,
    "forward must not select the previous reverse branch");
  require(recovery::nearestInPhase(plan, 2, 5, current) == 5,
    "second reverse must not jump to the last forward branch");
  require(recovery::nearestInPhase(plan, 3, 5, {-2, 0, 0}) == 6,
    "phase transition must advance into the new range");
  require(recovery::nearestInPhase(plan, 0, 2, current) == 2,
    "progress must not move backwards within a phase");
  require(recovery::nearestInPhase(plan, 0, 8, current) == 0,
    "out-of-range progress must restart at the current phase beginning");
  require(!recovery::nearestInPhase(plan, 4, 0, current), "invalid phase must be rejected");
  auto invalid = plan;
  invalid.phases[0].path_end = 0;
  require(!recovery::nearestInPhase(invalid, 0, 0, current), "empty range must be rejected");
  invalid.phases[0].path_end = 100;
  require(!recovery::nearestInPhase(invalid, 0, 0, current), "oversize range must be rejected");
  require(recovery::truncatePhasePath(plan, 3, 1.0), "valid truncation must succeed");
  checkPlan(plan);
  require(plan.path.size() == 8 && plan.phases[3].path_begin == 6,
    "truncation must use the selected phase, not the first reverse count");
  require(recovery::nearestInPhase(plan, 3, 8, current) == 7,
    "truncated endpoint must constrain progress");

  // Check metadata produced by both actual planners without ROS dependencies.
  recovery::Corridor corridor;
  for (int i = -20; i <= 100; ++i) {
    corridor.x.push_back(i); corridor.y.push_back(0);
    corridor.lo.push_back(-10); corridor.hi.push_back(10);
  }
  const recovery::ObstacleMap obstacles;
  const recovery::VehicleParams vehicle;
  checkPlan(recovery::plan(corridor, obstacles, vehicle, current));
  checkPlan(recovery::plan(corridor, obstacles, vehicle, current, 4, 16, -1));
  recovery::GoalPlanParams params;
  params.max_expand = 1000;
  checkPlan(recovery::planToGoal(corridor, obstacles, vehicle, current, {}, params));
  params.first_phase = -1;
  checkPlan(recovery::planToGoal(corridor, obstacles, vehicle, current, {}, params));

  // A rear vehicle in a contact chain must be allowed to make only the safe
  // partial reverse move.  Forcing a forward tail immediately drives it back
  // into the same stopped front vehicle and used to leave both cars with no plan.
  const std::vector<recovery::CarObstacle> stopped_front{{2.0, 0.0, "front"}};
  const auto reverse_only = recovery::plan(
    corridor, obstacles, vehicle, current, 0.0, 0.0, -1,
    0.0, 0.1, 0.75, 0.75, stopped_front, 0.0, 0.0, true);
  checkPlan(reverse_only);
  require(reverse_only.phases.size() == 1 && !reverse_only.phases.front().forward,
    "a safe reverse-only partial escape must be available for a blocked contact chain");
  std::vector<double> wall_trace(reverse_only.path.size(), 1000.0);
  std::vector<double> car_trace;
  for (const auto & pose : reverse_only.path) {
    car_trace.push_back(recovery::carClearanceAt(stopped_front.front(), vehicle, pose));
  }
  const std::vector<recovery::CarClearanceTrace> per_car{
    {"front", car_trace.front(), car_trace}};
  const auto partial_certificate = recovery::certifyEscapePlan(
    reverse_only, 1000.0, wall_trace, car_trace.front(), car_trace, {}, per_car);
  require(partial_certificate.valid,
    "the reverse-only partial escape must pass the same immutable runtime certificate");
  require(car_trace.back() >= car_trace.front() + 0.10,
    "the reverse-only partial escape must materially reduce the front-car overlap");
  std::cout << "phase path invariants passed\n";
}
