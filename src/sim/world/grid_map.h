#pragma once
#include <vector>
#include <cstdint>
#include "../math/vec2.h"
#include "../rng.h"

struct GridCell { int x; int y; };

struct GridMap {
  int w = 0;
  int h = 0;
  // 0 free, 1 occupied
  std::vector<uint8_t> occ;

  explicit GridMap(int w_, int h_);

  void clear(uint8_t v);
  bool in_bounds(int x, int y) const;
  bool is_occupied(int x, int y) const;

  void add_rect_obstacle(int x0, int y0, int x1, int y1);
  void add_random_rect_obstacles(Rng& rng, int count, int minw, int maxw, int minh, int maxh);

  GridCell world_to_cell(const Vec2& p) const;
  Vec2 cell_center_world(const GridCell& c) const;
};
