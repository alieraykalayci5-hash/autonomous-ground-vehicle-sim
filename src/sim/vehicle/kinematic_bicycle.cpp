#include "kinematic_bicycle.h"
#include <cmath>

static double clampd(double v, double lo, double hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

void KinematicBicycleModel::step(VehicleState& s, const VehicleControl& u, double dt) const {
  double steer = clampd(u.steer_rad, -p.max_steer_rad, p.max_steer_rad);
  double accel = clampd(u.accel, -p.max_accel, p.max_accel);

  s.v += accel * dt;
  s.v = clampd(s.v, 0.0, p.max_speed);

  const double beta = std::tan(steer) / p.wheelbase_L;
  s.yaw += s.v * beta * dt;

  s.x += s.v * std::cos(s.yaw) * dt;
  s.y += s.v * std::sin(s.yaw) * dt;
}
