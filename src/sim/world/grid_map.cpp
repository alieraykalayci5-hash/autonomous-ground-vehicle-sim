#include "grid_map.h"
#include <algorithm>
#include <cmath>

GridMap::GridMap(int w_, int h_) : w(w_), h(h_), occ((size_t)w_*(size_t)h_, 0) {}

void GridMap::clear(uint8_t v) {
  std::fill(occ.begin(), occ.end(), v);
}

bool GridMap::in_bounds(int x, int y) const {
  return (x >= 0 && x < w && y >= 0 && y < h);
}

bool GridMap::is_occupied(int x, int y) const {
  if (!in_bounds(x,y)) return true; // outside treated as occupied
  return occ[(size_t)y*(size_t)w + (size_t)x] != 0;
}

void GridMap::add_rect_obstacle(int x0, int y0, int x1, int y1) {
  int ax0 = std::min(x0, x1);
  int ax1 = std::max(x0, x1);
  int ay0 = std::min(y0, y1);
  int ay1 = std::max(y0, y1);

  ax0 = std::max(0, ax0); ay0 = std::max(0, ay0);
  ax1 = std::min(w-1, ax1); ay1 = std::min(h-1, ay1);

  for (int y = ay0; y <= ay1; y++) {
    for (int x = ax0; x <= ax1; x++) {
      occ[(size_t)y*(size_t)w + (size_t)x] = 1;
    }
  }
}

void GridMap::add_random_rect_obstacles(Rng& rng, int count, int minw, int maxw, int minh, int maxh) {
  for (int i = 0; i < count; i++) {
    int rw = minw + (int)(rng.u32() % (uint32_t)(maxw - minw + 1));
    int rh = minh + (int)(rng.u32() % (uint32_t)(maxh - minh + 1));
    int x0 = (int)(rng.u32() % (uint32_t)w);
    int y0 = (int)(rng.u32() % (uint32_t)h);
    int x1 = x0 + rw;
    int y1 = y0 + rh;
    add_rect_obstacle(x0, y0, x1, y1);
  }
}

GridCell GridMap::world_to_cell(const Vec2& p) const {
  int x = (int)std::floor(p.x);
  int y = (int)std::floor(p.y);
  return {x,y};
}

Vec2 GridMap::cell_center_world(const GridCell& c) const {
  return Vec2((double)c.x + 0.5, (double)c.y + 0.5);
}
