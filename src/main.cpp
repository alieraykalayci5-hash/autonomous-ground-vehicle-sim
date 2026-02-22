#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <filesystem>
#include <iostream>

#include "sim/config.h"
#include "sim/rng.h"
#include "sim/fnv1a.h"
#include "sim/csv.h"
#include "sim/math/vec2.h"
#include "sim/world/grid_map.h"
#include "sim/vehicle/vehicle_state.h"
#include "sim/vehicle/kinematic_bicycle.h"
#include "sim/planning/astar.h"
#include "sim/control/pid.h"
#include "sim/control/pure_pursuit.h"
#include "sim/tracking/alpha_beta_tracker.h"

namespace fs = std::filesystem;

static bool starts_with(const std::string& s, const char* pfx) { return s.rfind(pfx, 0) == 0; }

static std::string get_arg(int argc, char** argv, const char* key, const char* defv) {
  for (int i = 1; i < argc; i++) {
    if (starts_with(argv[i], key)) {
      const char* eq = std::strchr(argv[i], '=');
      if (eq) return std::string(eq + 1);
      if (i + 1 < argc) return std::string(argv[i + 1]);
    }
  }
  return std::string(defv);
}

static int get_arg_int(int argc, char** argv, const char* key, int defv) {
  auto s = get_arg(argc, argv, key, "");
  if (s.empty()) return defv;
  return std::atoi(s.c_str());
}

static double get_arg_double(int argc, char** argv, const char* key, double defv) {
  auto s = get_arg(argc, argv, key, "");
  if (s.empty()) return defv;
  return std::atof(s.c_str());
}

static double clampd(double v, double lo, double hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

struct TargetTruth { Vec2 pos; Vec2 vel; };

static TargetTruth target_truth_at(double t, const Vec2& p0, const Vec2& v) { return { p0 + v * t, v }; }

static void write_map_csv(const fs::path& out_dir, const GridMap& map) {
  CSVWriter w((out_dir / "map.csv").string());
  w.write_row({"x","y","occ"});
  for (int y = 0; y < map.h; y++) {
    for (int x = 0; x < map.w; x++) {
      w.write_row({std::to_string(x), std::to_string(y), std::to_string(map.occ[y*map.w + x])});
    }
  }
}

static uint64_t hash_file_bytes(const fs::path& p) {
  FILE* f = std::fopen(p.string().c_str(), "rb");
  if (!f) return 0;
  uint64_t h = fnv1a64_init();
  unsigned char buf[4096];
  while (true) {
    size_t n = std::fread(buf, 1, sizeof(buf), f);
    if (n == 0) break;
    h = fnv1a64_update(h, buf, n);
  }
  std::fclose(f);
  return h;
}

static void usage() {
  std::cout <<
    "agv_sim --mode static|moving|replan_demo --seed N --steps N --dt S\n"
    "       --map_w N --map_h N --noise_pos S --target_vx S --target_vy S\n"
    "       --lookahead S --max_speed S --out DIR --hash 0|1\n";
}

int main(int argc, char** argv) {
  const std::string mode = get_arg(argc, argv, "--mode", "static");
  const uint32_t seed = (uint32_t)get_arg_int(argc, argv, "--seed", 123);
  const int steps = get_arg_int(argc, argv, "--steps", 1200);
  const double dt = get_arg_double(argc, argv, "--dt", 0.02);
  const int map_w = get_arg_int(argc, argv, "--map_w", 80);
  const int map_h = get_arg_int(argc, argv, "--map_h", 60);
  const double noise_pos = get_arg_double(argc, argv, "--noise_pos", 2.0);
  const double target_vx = get_arg_double(argc, argv, "--target_vx", (mode=="moving") ? -1.5 : 0.0);
  const double target_vy = get_arg_double(argc, argv, "--target_vy", (mode=="moving") ?  0.5 : 0.0);
  const double lookahead = get_arg_double(argc, argv, "--lookahead", 3.5);
  const double max_speed = get_arg_double(argc, argv, "--max_speed", 8.0);
  const std::string out_dir_s = get_arg(argc, argv, "--out", "out_run");
  const int hash_on = get_arg_int(argc, argv, "--hash", 1);

  if (mode != "static" && mode != "moving" && mode != "replan_demo") {
    usage();
    return 2;
  }

  fs::path out_dir(out_dir_s);
  fs::create_directories(out_dir);

  Rng rng(seed);

  // World
  GridMap map(map_w, map_h);
  map.clear(0);
  map.add_rect_obstacle(10, 10, 14, 40);
  map.add_rect_obstacle(25,  0, 28, 32);
  map.add_rect_obstacle(40, 22, 60, 25);
  map.add_random_rect_obstacles(rng, 6, 3, 8, 3, 10);
  write_map_csv(out_dir, map);

  // Vehicle
  VehicleState vs{};
  vs.x = 5.0; vs.y = 5.0; vs.yaw = 0.0; vs.v = 0.0;

  KinematicBicycleParams vehp{};
  vehp.wheelbase_L = 2.6;
  vehp.max_steer_rad = deg2rad(32.0);
  vehp.max_accel = 3.5;
  vehp.max_speed = max_speed;
  KinematicBicycleModel veh(vehp);

  // Target
  Vec2 target_p0((double)map_w - 8.0, (double)map_h - 8.0);
  Vec2 target_v(target_vx, target_vy);

  // Tracking
  AlphaBetaTracker tracker;
  tracker.reset(target_p0, target_v);

  // Control
  PurePursuit pp;
  pp.lookahead = lookahead;
  pp.wheelbase_L = vehp.wheelbase_L;

  PID speed_pid;
  speed_pid.kp = 1.2; speed_pid.ki = 0.15; speed_pid.kd = 0.05; speed_pid.i_limit = 10.0;

  // Outputs
  CSVWriter truth_w((out_dir / "truth.csv").string());
  truth_w.write_row({"t","target_x","target_y","target_vx","target_vy"});

  CSVWriter state_w((out_dir / "state.csv").string());
  state_w.write_row({"t","x","y","yaw","v"});

  CSVWriter plan_w((out_dir / "plan.csv").string());
  plan_w.write_row({"plan_id","idx","x","y"});

  CSVWriter ctrl_w((out_dir / "control.csv").string());
  ctrl_w.write_row({"t","steer_rad","accel","v_ref","cte"});

  CSVWriter evt_w((out_dir / "events.csv").string());
  evt_w.write_row({"t","event","detail"});

  CSVWriter est_w((out_dir / "target_est.csv").string());
  est_w.write_row({"t","zx","zy","xhat","yhat","vxhat","vyhat"});

  auto emit_event = [&](double t, const char* ev, const std::string& detail){
    evt_w.write_row({to_string_fixed(t, 4), ev, detail});
  };

  // Planning helper
  int plan_id = 0;
  std::vector<GridCell> path_cells;
  std::vector<Vec2> path_world;
  Vec2 goal_world = target_p0;

  auto plan_now = [&](const Vec2& goal)->bool {
    GridCell s = map.world_to_cell(Vec2(vs.x, vs.y));
    GridCell g = map.world_to_cell(goal);

    if (!map.in_bounds(s.x, s.y) || !map.in_bounds(g.x, g.y)) return false;

    if (map.is_occupied(g.x, g.y)) {
      bool found = false;
      for (int r = 1; r <= 6 && !found; r++) {
        for (int dy = -r; dy <= r && !found; dy++) {
          for (int dx = -r; dx <= r && !found; dx++) {
            int nx = g.x + dx, ny = g.y + dy;
            if (map.in_bounds(nx, ny) && !map.is_occupied(nx, ny)) {
              g = {nx, ny};
              found = true;
            }
          }
        }
      }
      if (!found) return false;
    }

    AStarPlanner planner;
    planner.allow_diagonal = true;
    planner.corner_cutting = false;

    bool ok = planner.plan(map, s, g, path_cells);
    if (!ok) return false;

    path_world.clear();
    path_world.reserve(path_cells.size());
    for (auto& c : path_cells) path_world.push_back(map.cell_center_world(c));

    for (size_t i = 0; i < path_world.size(); i++) {
      plan_w.write_row({
        std::to_string(plan_id),
        std::to_string((int)i),
        to_string_fixed(path_world[i].x, 4),
        to_string_fixed(path_world[i].y, 4)
      });
    }
    plan_id++;
    return true;
  };

  bool tracking_enabled = (mode == "moving");
  if (!plan_now(goal_world)) {
    emit_event(0.0, "NO_PATH", "initial");
    std::fprintf(stderr, "NO_PATH\n");
    return 3;
  }

  const int inject_step = (mode=="replan_demo") ? (steps/3) : -1;
  const double reach_dist = 2.0;

  GridCell last_goal_cell{-9999,-9999};
  double t = 0.0;

  for (int k = 0; k < steps; k++, t += dt) {
    // Truth
    TargetTruth tt = target_truth_at(t, target_p0, target_v);
    truth_w.write_row({
      to_string_fixed(t, 4),
      to_string_fixed(tt.pos.x, 4),
      to_string_fixed(tt.pos.y, 4),
      to_string_fixed(tt.vel.x, 4),
      to_string_fixed(tt.vel.y, 4)
    });

    // Sensor + tracking
    Vec2 z = tt.pos;
    if (tracking_enabled) {
      z = Vec2(tt.pos.x + rng.gaussian(0.0, noise_pos),
               tt.pos.y + rng.gaussian(0.0, noise_pos));
      tracker.update(z, dt);
    } else {
      tracker.reset(tt.pos, tt.vel);
    }

    est_w.write_row({
      to_string_fixed(t, 4),
      to_string_fixed(z.x, 4),
      to_string_fixed(z.y, 4),
      to_string_fixed(tracker.pos_hat().x, 4),
      to_string_fixed(tracker.pos_hat().y, 4),
      to_string_fixed(tracker.vel_hat().x, 4),
      to_string_fixed(tracker.vel_hat().y, 4)
    });

    // Replan injection
    if (k == inject_step) {
      int wx0 = map_w/2 - 1, wx1 = map_w/2 + 1;
      map.add_rect_obstacle(wx0, 5, wx1, map_h-6);
      emit_event(t, "REPLAN", "inject_wall");
      if (!plan_now(goal_world)) {
        emit_event(t, "NO_PATH", "after_inject");
        break;
      }
    }

    // Goal select
    if (tracking_enabled) {
      double tlead = 1.2;
      goal_world = tracker.pos_hat() + tracker.vel_hat() * tlead;
    } else {
      goal_world = tt.pos;
    }

    // Replan if goal cell changed (moving)
    GridCell gc = map.world_to_cell(goal_world);
    if (tracking_enabled && (gc.x != last_goal_cell.x || gc.y != last_goal_cell.y)) {
      emit_event(t, "REPLAN", "goal_moved");
      (void)plan_now(goal_world);
      last_goal_cell = gc;
    }

    // Control
    double cte = 0.0;
    double steer = pp.compute_steer(Vec2(vs.x, vs.y), vs.yaw, path_world, cte);
    steer = clampd(steer, -vehp.max_steer_rad, vehp.max_steer_rad);

    double dist_goal = (Vec2(vs.x, vs.y) - goal_world).norm();
    double v_ref = (dist_goal < 10.0) ? clampd(dist_goal * 0.8, 0.0, vehp.max_speed) : vehp.max_speed;

    double accel_cmd = speed_pid.step(v_ref - vs.v, dt);
    accel_cmd = clampd(accel_cmd, -vehp.max_accel, vehp.max_accel);

    VehicleControl u{};
    u.steer_rad = steer;
    u.accel = accel_cmd;
    veh.step(vs, u, dt);

    // Collision
    GridCell vc = map.world_to_cell(Vec2(vs.x, vs.y));
    bool collision = (!map.in_bounds(vc.x, vc.y)) || map.is_occupied(vc.x, vc.y);
    if (collision) {
      emit_event(t, "COLLISION", "vehicle_in_occupied");
      vs.v = 0.0;
    }

    state_w.write_row({
      to_string_fixed(t, 4),
      to_string_fixed(vs.x, 4),
      to_string_fixed(vs.y, 4),
      to_string_fixed(vs.yaw, 6),
      to_string_fixed(vs.v, 4)
    });

    ctrl_w.write_row({
      to_string_fixed(t, 4),
      to_string_fixed(steer, 6),
      to_string_fixed(accel_cmd, 6),
      to_string_fixed(v_ref, 4),
      to_string_fixed(cte, 4)
    });

    if (dist_goal < reach_dist) {
      emit_event(t, "REACHED", "goal");
      break;
    }
  }

  // Hash: hash of hashes of output files
  uint64_t h = fnv1a64_init();
  const char* files[] = {"truth.csv","state.csv","plan.csv","control.csv","events.csv","target_est.csv","map.csv"};
  for (auto* fn : files) {
    uint64_t fh = hash_file_bytes(out_dir / fn);
    h = fnv1a64_update_u64(h, fh);
  }

  if (hash_on) std::fprintf(stderr, "FNV1A64=%016llx\n", (unsigned long long)h);
  return 0;
}