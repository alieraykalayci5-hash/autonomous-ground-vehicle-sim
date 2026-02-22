#include "astar.h"
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

struct Node {
  int x, y;
  double f;
  double g;
};

struct NodeCmp {
  bool operator()(const Node& a, const Node& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g < b.g;
    if (a.y != b.y) return a.y > b.y;
    return a.x > b.x;
  }
};

static double heuristic(int x, int y, int gx, int gy) {
  double dx = (double)(gx - x);
  double dy = (double)(gy - y);
  return std::sqrt(dx*dx + dy*dy);
}

bool AStarPlanner::plan(const GridMap& map, GridCell start, GridCell goal, std::vector<GridCell>& out_path) {
  out_path.clear();

  if (map.is_occupied(start.x, start.y)) return false;
  if (map.is_occupied(goal.x, goal.y)) return false;

  const int W = map.w, H = map.h;
  const int N = W * H;
  auto idx = [&](int x, int y)->int { return y*W + x; };

  std::vector<double> gscore((size_t)N, std::numeric_limits<double>::infinity());
  std::vector<int> parent((size_t)N, -1);
  std::vector<uint8_t> closed((size_t)N, 0);

  std::priority_queue<Node, std::vector<Node>, NodeCmp> open;

  int sidx = idx(start.x, start.y);
  gscore[(size_t)sidx] = 0.0;
  open.push({start.x, start.y, heuristic(start.x,start.y,goal.x,goal.y), 0.0});

  const int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
  const int dirs8[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

  while (!open.empty()) {
    Node cur = open.top(); open.pop();
    if (!map.in_bounds(cur.x, cur.y)) continue;
    int cidx = idx(cur.x, cur.y);
    if (closed[(size_t)cidx]) continue;
    closed[(size_t)cidx] = 1;

    if (cur.x == goal.x && cur.y == goal.y) {
      std::vector<GridCell> rev;
      int p = cidx;
      while (p != -1) {
        int px = p % W;
        int py = p / W;
        rev.push_back({px, py});
        p = parent[(size_t)p];
      }
      std::reverse(rev.begin(), rev.end());
      out_path = std::move(rev);
      return true;
    }

    const int (*dirs)[2] = allow_diagonal ? dirs8 : dirs4;
    const int nd = allow_diagonal ? 8 : 4;

    for (int i = 0; i < nd; i++) {
      int nx = cur.x + dirs[i][0];
      int ny = cur.y + dirs[i][1];
      if (!map.in_bounds(nx, ny)) continue;
      if (map.is_occupied(nx, ny)) continue;

      if (allow_diagonal && !corner_cutting && std::abs(dirs[i][0])==1 && std::abs(dirs[i][1])==1) {
        int ax = cur.x + dirs[i][0];
        int ay = cur.y;
        int bx = cur.x;
        int by = cur.y + dirs[i][1];
        if (map.is_occupied(ax, ay) || map.is_occupied(bx, by)) continue;
      }

      int nidx = idx(nx, ny);
      if (closed[(size_t)nidx]) continue;

      double step_cost = (std::abs(dirs[i][0]) + std::abs(dirs[i][1]) == 2) ? std::sqrt(2.0) : 1.0;
      double ng = gscore[(size_t)cidx] + step_cost;

      if (ng < gscore[(size_t)nidx]) {
        gscore[(size_t)nidx] = ng;
        parent[(size_t)nidx] = cidx;
        double h = heuristic(nx, ny, goal.x, goal.y);
        open.push({nx, ny, ng + h, ng});
      }
    }
  }

  return false;
}
