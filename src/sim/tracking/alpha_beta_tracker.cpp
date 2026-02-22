#include "alpha_beta_tracker.h"

void AlphaBetaTracker::reset(const Vec2& pos, const Vec2& vel) {
  xh = pos;
  vh = vel;
}

void AlphaBetaTracker::update(const Vec2& z_pos, double dt) {
  Vec2 x_pred = xh + vh * dt;
  Vec2 v_pred = vh;
  Vec2 r = z_pos - x_pred;
  xh = x_pred + alpha * r;
  vh = v_pred + (beta / dt) * r;
}
