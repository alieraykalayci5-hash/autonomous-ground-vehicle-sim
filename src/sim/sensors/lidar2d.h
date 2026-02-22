#pragma once
#include <vector>
#include "../math/vec2.h"
#include "../world/grid_map.h"

// Simple 2D lidar: casts rays in the occupancy grid and returns hit distances.
struct Lidar2D {
  int beams = 31;                 // odd number recommended
  double fov_rad = 1.57079632679; // 90 deg
  double max_range = 12.0;        // cells
  double step = 0.2;              // ray marching step (cells)

  // Returns per-beam range in [0,max_range]. If no hit, range=max_range.
  std::vector<double> scan(const GridMap& map, const Vec2& sensor_pos, double sensor_yaw) const;

  // True if any hit within threshold in a forward cone (centered around yaw).
  bool obstacle_ahead(const GridMap& map, const Vec2& sensor_pos, double sensor_yaw,
                      double forward_cone_rad, double threshold_range) const;
};