#pragma once

struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0; // rad
  double v = 0.0;   // m/s
};

struct VehicleControl {
  double steer_rad = 0.0;
  double accel = 0.0;
};
