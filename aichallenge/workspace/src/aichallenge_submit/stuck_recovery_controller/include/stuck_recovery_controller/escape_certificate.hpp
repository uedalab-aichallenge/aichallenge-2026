#ifndef STUCK_RECOVERY_CONTROLLER__ESCAPE_CERTIFICATE_HPP_
#define STUCK_RECOVERY_CONTROLLER__ESCAPE_CERTIFICATE_HPP_

#include "stuck_recovery_controller/recovery_planner.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace recovery
{

// A plan may have to start inside the map's wall/vehicle envelope.  Such a plan
// is safe only as one indivisible escape: it may remain negative while leaving
// the overlap, but it must never become materially worse. Walls must finish free;
// an initial vehicle overlap may finish with a certified penetration improvement.
// These tolerances are deliberately independent of controller timing so the
// same contract is used for goal, legacy and best-effort planner results.
struct EscapeCertificateOptions
{
  double plan_worsen_tolerance{0.02};
  // 処理内容を示す。
  double plan_escape_dip{0.30};
  double terminal_clearance{0.0};
  // Runtime uses measured clearance, not a nearest discrete model sample.  If
  // the plan starts overlapped it may not deepen that overlap; if it starts
  // clear it may not enter one.  The controller additionally ratchets progress
  // while it is still inside the wall.
  double runtime_start_worsen_tolerance{0.05};
  double runtime_progress_worsen_tolerance{0.20};
  double terminal_car_progress{0.10};
  double runtime_car_progress_worsen_tolerance{0.05};
};

struct CarClearanceTrace
{
  std::string vehicle_id;
  double start_clearance{0.0};
  std::vector<double> path_clearance;
};

struct CertifiedCarTrace
{
  std::string vehicle_id;
  bool starts_overlapping{false};
  double start_clearance{0.0};
  double minimum_clearance{0.0};
  double terminal_clearance{0.0};
  std::vector<double> path_clearance;
};

struct EscapeCertificate
{
  bool valid{false};
  bool starts_inside_wall{false};
  bool starts_overlapping_car{false};
  double start_wall_clearance{0.0};
  double minimum_wall_clearance{0.0};
  double terminal_wall_clearance{0.0};
  double start_car_clearance{0.0};
  double minimum_car_clearance{0.0};
  double terminal_car_clearance{0.0};
  std::vector<double> planned_wall_clearance;
  std::vector<double> planned_car_clearance;
  std::vector<CertifiedCarTrace> certified_cars;
  // Keep the exact accepted plan.  A valid flag alone is not a certificate if
  // a later post-process can resize or replace the path behind its back.
  std::vector<Pose> planned_path;
  std::vector<Phase> planned_phases;
  EscapeCertificateOptions options;
  std::string reason;

  // Absolute measured wall floor. path_index is retained for the stable API
  // and diagnostics, but safety does not depend on an ambiguous nearest sample.
  double runtimeWallFloor(std::size_t path_index) const;
  bool allowsRuntimeWallClearance(std::size_t path_index, double measured) const;
  bool matchesPlan(const Plan & plan) const;
};

// wall_clearances/car_clearances correspond one-for-one with plan.path.
// The caller supplies the separately measured starting clearances so a path
// beginning in an existing overlap can be distinguished from a path entering
// a new overlap.  No planner source receives an exception.
EscapeCertificate certifyEscapePlan(
  const Plan & plan,
  double start_wall_clearance,
  const std::vector<double> & wall_clearances,
  double start_car_clearance,
  const std::vector<double> & car_clearances,
  const EscapeCertificateOptions & options = {},
  const std::vector<CarClearanceTrace> & car_traces = {});

}  // namespace recovery

#endif  // STUCK_RECOVERY_CONTROLLER__ESCAPE_CERTIFICATE_HPP_
