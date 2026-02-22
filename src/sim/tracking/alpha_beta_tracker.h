#pragma once
#include "../math/vec2.h"

// Minimal deterministic tracker: alpha-beta filter for position + velocity (2D)
class AlphaBetaTracker {
  Vec2 xh;
  Vec2 vh;

public:
  double alpha = 0.85;
  double beta  = 0.005;

  void reset(const Vec2& pos, const Vec2& vel);
  void update(const Vec2& z_pos, double dt);

  Vec2 pos_hat() const { return xh; }
  Vec2 vel_hat() const { return vh; }
};
