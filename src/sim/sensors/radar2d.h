#pragma once
#include "../math/vec2.h"
#include "../rng.h"

// Minimal 2D radar-like sensor: position measurement with Gaussian noise.
// Optional detection probability (missed detections).
struct Radar2D {
  double sigma_pos = 2.0;   // stddev of position noise
  double p_detect  = 1.0;   // [0..1]

  // returns true if detection produced, false if missed
  bool measure(Rng& rng, const Vec2& truth_pos, Vec2& out_z) const;
};