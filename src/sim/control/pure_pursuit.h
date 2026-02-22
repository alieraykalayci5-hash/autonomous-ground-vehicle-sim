#pragma once
#include <vector>
#include "../math/vec2.h"

class PurePursuit {
public:
  double lookahead = 3.0;
  double wheelbase_L = 2.5;

  double compute_steer(const Vec2& pos, double yaw, const std::vector<Vec2>& path, double& cte_out) const;
};
