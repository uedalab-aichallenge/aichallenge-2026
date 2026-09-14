#include "stuck_recovery_controller/contact_lease.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace
{
void require(bool ok, const char * message)
{
  if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}

recovery::ContactLeaseDecision decide(
  std::vector<std::string> ids, const std::string & self, double time)
{
  return recovery::decideContactLease(std::move(ids), self, time, 4.0, 0.30);
}
}  // namespace

int main()
{
  for (const double time : {0.0, 0.5, 3.8, 4.5, 8.5}) {
    const auto a = decide({"d2", "d3"}, "d2", time);
    const auto b = decide({"d3", "d2", "d3"}, "d2", time);
    require(a.applicable == b.applicable && a.guard == b.guard &&
      a.is_owner == b.is_owner && a.slot == b.slot && a.lease_owner == b.lease_owner,
      "contact observation order and duplicates must not change the lease");
  }

  require(decide({"d3"}, "d2", 0.5).lease_owner == "d2", "slot zero must select d2");
  require(decide({"d3"}, "d2", 4.5).lease_owner == "d3", "slot one must select d3");
  require(decide({"d3"}, "d2", 8.5).lease_owner == "d2", "slot two must return to d2");
  require(decide({"d2", "d3"}, "d1", 0.5).lease_owner == "d1",
    "three-car rotation must start at the first sorted ID");
  require(decide({"d2", "d3"}, "d1", 4.5).lease_owner == "d2",
    "three-car rotation must advance one owner per slot");
  require(decide({"d2", "d3"}, "d1", 8.5).lease_owner == "d3",
    "three-car rotation must reach every owner");

  for (const double time : {0.0, 0.30, 3.70, 4.0}) {
    require(decide({"d3"}, "d2", time).guard, "both lease edges must be guarded inclusively");
  }
  require(!decide({"d3"}, "d2", 0.300001).guard,
    "time just after the leading guard must be movable");
  require(!decide({"d3"}, "d2", 3.699999).guard,
    "time just before the trailing guard must be movable");

  const auto alone = decide({}, "d2", 1.0);
  require(!alone.applicable && alone.is_owner, "a lone vehicle must not need a lease");
  const auto anonymous = decide({"d2"}, "", 1.0);
  require(!anonymous.applicable && !anonymous.is_owner, "an empty self ID must not claim a lease");
  const auto negative_time = decide({"d3"}, "d2", -1.0);
  require(negative_time.applicable && negative_time.guard && !negative_time.is_owner,
    "invalid negative time must fail closed");

  std::cout << "contact lease invariants passed\n";
}
