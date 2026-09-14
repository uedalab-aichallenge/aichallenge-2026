#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace recovery
{

struct ContactLeaseDecision
{
  bool applicable{false};
  bool guard{false};
  bool is_owner{false};
  std::uint64_t slot{0};
  double slot_position{0.0};
  std::string lease_owner;
};

// Every controller must derive the same mover from the same contact component,
// regardless of V2X observation order.  Invalid clock/lease inputs fail closed.
inline ContactLeaseDecision decideContactLease(
  std::vector<std::string> contact_ids, const std::string & self_id,
  double time_sec, double slot_sec, double guard_sec)
{
  ContactLeaseDecision result;
  if (self_id.empty()) {
    return result;
  }
  contact_ids.push_back(self_id);
  contact_ids.erase(
    std::remove(contact_ids.begin(), contact_ids.end(), std::string{}), contact_ids.end());
  std::sort(contact_ids.begin(), contact_ids.end());
  contact_ids.erase(std::unique(contact_ids.begin(), contact_ids.end()), contact_ids.end());
  if (contact_ids.size() < 2) {
    result.is_owner = true;
    return result;
  }

  result.applicable = true;
  if (!std::isfinite(time_sec) || time_sec < 0.0 || !std::isfinite(slot_sec) ||
    !std::isfinite(guard_sec) || slot_sec <= 0.0 || guard_sec < 0.0 ||
    guard_sec * 2.0 >= slot_sec)
  {
    result.guard = true;
    return result;
  }

  const double slot_floor = std::floor(time_sec / slot_sec);
  result.slot = static_cast<std::uint64_t>(slot_floor);
  result.slot_position = time_sec - slot_floor * slot_sec;
  // Normalize only floating-point roundoff at a slot boundary.
  if (result.slot_position < 0.0) {
    result.slot_position = 0.0;
  } else if (result.slot_position >= slot_sec) {
    result.slot_position = 0.0;
    ++result.slot;
  }
  result.lease_owner = contact_ids[result.slot % contact_ids.size()];
  result.guard = result.slot_position <= guard_sec ||
    result.slot_position >= slot_sec - guard_sec;
  result.is_owner = !result.guard && result.lease_owner == self_id;
  return result;
}

}  // namespace recovery
