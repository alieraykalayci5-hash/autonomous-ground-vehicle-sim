#pragma once
#include <vector>
#include "../world/grid_map.h"

class AStarPlanner {
public:
  bool allow_diagonal = true;
  bool corner_cutting = false;

  bool plan(const GridMap& map, GridCell start, GridCell goal, std::vector<GridCell>& out_path);
};
