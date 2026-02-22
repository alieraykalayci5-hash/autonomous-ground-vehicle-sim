# Autonomous Ground Vehicle Simulation (C++17)

## Demo Output

### Replanning Door-Passage Scenario

![Trajectory](docs/traj.png)
![Speed](docs/speed.png)

Deterministic 2D autonomous ground vehicle simulation in modern C++ integrating:

- Grid-based world model
- A* path planning
- Kinematic bicycle vehicle model
- Pure Pursuit steering + PID speed control
- GPS, Radar, LiDAR sensor simulation
- Target tracking (deterministic Alpha-Beta filter)
- Dynamic replanning demo (door-passage scenario)
- Golden-hash smoke tests (FNV-1a 64-bit)

This project is designed as a **systems-oriented real-time autonomy demo**:
fixed timestep simulation, deterministic RNG, CSV logging, and regression-detecting smoke tests.

---

# 1. Key Capabilities

## Deterministic Simulation Core

- Fixed timestep loop (`--dt`, default: 0.02s)
- Deterministic RNG (`--seed`)
- All outputs logged to CSV
- End-of-run FNV-1a 64-bit hash
- Smoke test verifies bit-level determinism

Running the same command with the same seed always produces identical hashes.

---

## World Model

- 2D occupancy grid (free / occupied cells)
- Deterministic obstacle placement
- Optional obstacle injection (replanning demo)
- Collision detection:
  - Vehicle enters occupied cell → collision event
  - Recovery + replanning triggered

---

## Vehicle Model

State:

(x, y, yaw, v)


Control:

steer_rad, accel


Model:
- Kinematic Bicycle
- Bounded steering angle
- Acceleration + max speed constraints

---

## Planning

Planner:
- A* grid search
- Diagonal motion allowed
- No corner cutting

Replanning triggers:
- Moving goal (tracking mode)
- Collision recovery
- Obstacle injection demo
- Door alignment transitions

---

## Control

### Steering
Pure Pursuit:
- Waypoint-based tracking
- Lookahead distance configurable

### Speed
PID controller:
- kp / ki / kd
- Integral windup limiting
- Acceleration clamping

---

## Sensors

### GPS2D
- Noisy vehicle position
- `sigma_pos`
- `p_fix` (probability of valid fix)

### Radar2D
- Noisy target position
- `sigma_pos`
- `p_detect`

### Lidar2D
- Beam-based sampling
- Forward obstacle detection
- Used as replanning trigger

---

## Target Tracking

- Alpha-Beta filter (deterministic)
- Estimates:
  - x̂, ŷ
  - v̂x, v̂y

Produces:

target_est.csv


This module demonstrates lightweight real-time state estimation.

---

# 2. Run Modes

## Static Target (Baseline Navigation)

```bash
./build/agv_sim --mode static --seed 123 --out out_static --hash 1

Vehicle navigates to a fixed goal.

Moving Target (Tracking + Intercept)
./build/agv_sim --mode moving --seed 123 --noise_pos 2.0 --p_detect 1.0 --out out_moving --hash 1

Vehicle predicts target future position and intercepts.

Replanning Demo (Door-Passage Scenario)
./build/agv_sim --mode replan_demo --seed 123 --out out_replan --hash 1

Scenario:

Vehicle navigates toward goal

Wall injected dynamically

Door carved into wall

Vehicle:

Replans to door approach

Aligns

Crosses

Replans to final goal

Reaches goal

Deterministic event sequence:

inject_wall_door

snap_to_approach

door_aligned

door_reached

REACHED goal

3. Output Files

Each run generates:

File	Description
truth.csv	Ground truth target state
state.csv	Vehicle state history
plan.csv	Planned waypoints (plan_id indexed)
control.csv	Steering / accel / tracking error
events.csv	Replans, collisions, status
target_est.csv	Tracker output
map.csv	Occupancy grid dump
gps.csv	GPS measurements
lidar.csv	LiDAR diagnostics
4. Plotting

Install dependencies:

py -m pip install -r tools\requirements.txt

Generate plots:

py tools\plot_run.py --in out_replan --out plots

Generated:

traj.png

speed.png

gps.png

lidar.png

5. Determinism & Smoke Test

Golden-hash regression test:

./scripts/smoke.sh
./scripts/smoke.sh

Second run must print:

[SMOKE] PASS

Current baseline hashes (seed=123):

static = c52c178345eb6a0a
moving = a1a5540b1e5e5be3
replan = e08f9d2c8ddeefbc

If simulation logic changes:

Delete scripts/expected.txt

Run smoke twice to re-baseline

6. Architecture Overview
Simulation Loop (fixed dt)
 ├── Sensor simulation
 ├── Target tracking
 ├── Goal selection
 ├── Planning (A*)
 ├── Pure Pursuit steering
 ├── PID speed control
 ├── Vehicle integration
 ├── Collision detection
 ├── Event logging
 └── CSV output

Determinism ensured via:

Fixed timestep

No system time usage

Seeded RNG

Ordered CSV writes

7. Design Goals

This project demonstrates:

Real-time control integration

Planning + tracking coupling

Deterministic reproducibility

Systems-level architecture

Debuggable logging

Regression safety via hashing

It is intentionally lightweight but architecturally extensible.

8. Limitations

Grid-based map (not continuous occupancy)

Kinematic vehicle model (no tire slip dynamics)

Single-target tracking

Simplified LiDAR model

No dynamic obstacle objects (injection demo only)

9. Future Extensions

Extended Kalman Filter

Multi-target tracking + association

Dynamic obstacle objects

Stanley controller alternative

Path smoothing

CI pipeline with automatic smoke verification

10. Build (MSYS2 UCRT64)
cmake -S . -B build -G Ninja
cmake --build build -j

Executable:

build/agv_sim.exe

MIT License

**Ali Eray Kalaycı**  
Computer Engineering  
Focus: Real-Time Systems, Tracking & Estimation, Autonomous Systems