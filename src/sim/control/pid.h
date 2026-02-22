#pragma once

struct PID {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double i_limit = 0.0;

  double i = 0.0;
  double prev_e = 0.0;
  bool has_prev = false;

  void reset() { i = 0.0; prev_e = 0.0; has_prev = false; }

  double step(double e, double dt) {
    i += e * dt;
    if (i_limit > 0.0) {
      if (i > i_limit) i = i_limit;
      if (i < -i_limit) i = -i_limit;
    }
    double de = 0.0;
    if (has_prev) de = (e - prev_e) / dt;
    prev_e = e;
    has_prev = true;
    return kp*e + ki*i + kd*de;
  }
};
