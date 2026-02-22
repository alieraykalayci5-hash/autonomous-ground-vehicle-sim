#pragma once
#include "vehicle_state.h"

struct KinematicBicycleParams {
  double wheelbase_L = 2.5;
  double max_steer_rad = 0.6;
  double max_accel = 3.0;
  double max_speed = 10.0;
};

class KinematicBicycleModel {
  KinematicBicycleParams p;

public:
  explicit KinematicBicycleModel(const KinematicBicycleParams& params) : p(params) {}
  void step(VehicleState& s, const VehicleControl& u, double dt) const;
};
