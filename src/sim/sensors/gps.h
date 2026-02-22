#pragma once
#include "../math/vec2.h"
#include "../rng.h"
#include "../vehicle/vehicle_state.h"

// Minimal GPS-like sensor: noisy position measurement of the vehicle.
// Deterministic via provided RNG.
struct GPS2D {
  double sigma_pos = 0.5;  // stddev position noise (meters)
  double p_fix     = 1.0;  // probability of having a fix (missed fix if <1)

  // Returns true if measurement available, false if no-fix.
  bool measure(Rng& rng, const VehicleState& truth_state, Vec2& out_pos) const;
};