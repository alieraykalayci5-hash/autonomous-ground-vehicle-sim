#include "pure_pursuit.h"
#include <cmath>

static Vec2 rot_to_vehicle(const Vec2& p_world, const Vec2& veh_pos, double yaw) {
  double dx = p_world.x - veh_pos.x;
  double dy = p_world.y - veh_pos.y;
  double cy = std::cos(-yaw);
  double sy = std::sin(-yaw);
  return Vec2(dx*cy - dy*sy, dx*sy + dy*cy);
}

double PurePursuit::compute_steer(const Vec2& pos, double yaw, const std::vector<Vec2>& path, double& cte_out) const {
  cte_out = 0.0;
  if (path.empty()) return 0.0;

  Vec2 look = path.back();
  double best_d = 1e30;
  for (size_t i = 0; i < path.size(); i++) {
    double d = (path[i] - pos).norm();
    if (d < best_d) best_d = d;
    if (d >= lookahead) { look = path[i]; break; }
  }
  cte_out = best_d;

  Vec2 lv = rot_to_vehicle(look, pos, yaw);
  if (lv.x <= 1e-6) return 0.0;

  double Ld2 = lv.x*lv.x + lv.y*lv.y;
  if (Ld2 < 1e-9) return 0.0;
  double kappa = 2.0 * lv.y / Ld2;
  return std::atan(wheelbase_L * kappa);
}
