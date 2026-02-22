#include "lidar2d.h"
#include <cmath>
#include <algorithm>

static double clampd(double v, double lo, double hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

std::vector<double> Lidar2D::scan(const GridMap& map, const Vec2& sensor_pos, double sensor_yaw) const {
  int n = std::max(1, beams);
  std::vector<double> ranges((size_t)n, max_range);

  double half = 0.5 * fov_rad;
  for (int i = 0; i < n; i++) {
    double u = (n == 1) ? 0.0 : (double)i / (double)(n - 1); // 0..1
    double ang = sensor_yaw + (-half + u * fov_rad);

    double c = std::cos(ang);
    double s = std::sin(ang);

    double r = 0.0;
    double hit = max_range;

    while (r <= max_range) {
      Vec2 p(sensor_pos.x + c * r, sensor_pos.y + s * r);
      GridCell cell = map.world_to_cell(p);
      if (map.is_occupied(cell.x, cell.y)) { hit = r; break; }
      r += step;
    }

    ranges[(size_t)i] = clampd(hit, 0.0, max_range);
  }

  return ranges;
}

bool Lidar2D::obstacle_ahead(const GridMap& map, const Vec2& sensor_pos, double sensor_yaw,
                             double forward_cone_rad, double threshold_range) const {
  Lidar2D tmp = *this;
  tmp.fov_rad = forward_cone_rad;
  auto ranges = tmp.scan(map, sensor_pos, sensor_yaw);
  for (double r : ranges) {
    if (r <= threshold_range) return true;
  }
  return false;
}