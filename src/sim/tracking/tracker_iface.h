#pragma once
#include "../math/vec2.h"

// Tracker interface: position-only measurement -> state estimate (pos+vel)
struct ITracker2D {
  virtual ~ITracker2D() = default;

  virtual void reset(const Vec2& pos, const Vec2& vel) = 0;
  virtual void update(const Vec2& z_pos, double dt) = 0;

  virtual Vec2 pos_hat() const = 0;
  virtual Vec2 vel_hat() const = 0;
};