cat > README.md <<'EOF'
# Autonomous Ground Vehicle Sim (Deterministic)

Deterministic 2D autonomous ground vehicle simulation in **C++17** with:

- **World**: 2D occupancy grid + deterministic obstacle generation (seeded)
- **Planning**: A* (8-neighbor optional, no corner cutting by default)
- **Control**: Pure Pursuit steering + PID speed control
- **Tracking integration**: position-only noisy measurement + alpha-beta tracker (pos/vel estimate) + intercept goal
- **Real-time sim loop**: fixed timestep
- **Logs**: CSV outputs for reproducibility and analysis
- **Determinism**: FNV-1a 64-bit hash over run outputs + golden smoke test

This repo is designed as a systems-style demo: deterministic behavior, explicit boundaries, and regression detection.

## Build (Windows / MSYS2 UCRT64)

Install MSYS2 packages:

```bash
pacman -S --needed mingw-w64-ucrt-x86_64-toolchain mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja