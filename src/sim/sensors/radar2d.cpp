#include "radar2d.h"

bool Radar2D::measure(Rng& rng, const Vec2& truth_pos, Vec2& out_z) const {
  if (p_detect < 1.0) {
    double u = rng.uniform01();
    if (u > p_detect) return false;
  }

  out_z = Vec2(
    truth_pos.x + rng.gaussian(0.0, sigma_pos),
    truth_pos.y + rng.gaussian(0.0, sigma_pos)
  );
  return true;
}