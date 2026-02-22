#include "gps.h"

bool GPS2D::measure(Rng& rng, const VehicleState& truth_state, Vec2& out_pos) const {
  if (p_fix < 1.0) {
    double u = rng.uniform01();
    if (u > p_fix) return false;
  }

  out_pos = Vec2(
    truth_state.x + rng.gaussian(0.0, sigma_pos),
    truth_state.y + rng.gaussian(0.0, sigma_pos)
  );
  return true;
}