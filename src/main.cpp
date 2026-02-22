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

#include "sim/tracking/tracker_iface.h"
#include "sim/tracking/alpha_beta_tracker.h"

#include "sim/sensors/radar2d.h"
#include "sim/sensors/gps.h"
#include "sim/sensors/lidar2d.h"

namespace fs = std::filesystem;

static bool starts_with(const std::string& s, const char* pfx) { return s.rfind(pfx, 0) == 0; }
static bool has_key(int argc, char** argv, const char* key) {
  for (int i = 1; i < argc; i++) if (starts_with(argv[i], key)) return true;
  return false;
}

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
    "       --map_w N --map_h N --noise_pos S --p_detect S\n"
    "       --gps_sigma S --gps_p_fix S\n"
    "       --lidar_beams N --lidar_fov_deg S --lidar_range S --lidar_step S\n"
    "       --ahead_cone_deg S --ahead_thresh S\n"
    "       --target_vx S --target_vy S --lookahead S --max_speed S\n"
    "       --out DIR --hash 0|1\n";
}

// Adapter: AlphaBetaTracker -> ITracker2D
struct AlphaBetaAdapter final : public ITracker2D {
  AlphaBetaTracker impl;
  void reset(const Vec2& pos, const Vec2& vel) override { impl.reset(pos, vel); }
  void update(const Vec2& z_pos, double dt) override { impl.update(z_pos, dt); }
  Vec2 pos_hat() const override { return impl.pos_hat(); }
  Vec2 vel_hat() const override { return impl.vel_hat(); }
};

static bool is_cell_free(const GridMap& map, int x, int y) {
  return map.in_bounds(x, y) && !map.is_occupied(x, y);
}

static bool find_nearest_free_cell(const GridMap& map, GridCell start, int max_r, GridCell& out) {
  if (is_cell_free(map, start.x, start.y)) { out = start; return true; }
  for (int r = 1; r <= max_r; r++) {
    for (int dy = -r; dy <= r; dy++) {
      for (int dx = -r; dx <= r; dx++) {
        int nx = start.x + dx;
        int ny = start.y + dy;
        if (is_cell_free(map, nx, ny)) { out = {nx, ny}; return true; }
      }
    }
  }
  return false;
}

static GridMap build_nav_map_inflated(const GridMap& src, int r) {
  GridMap nav(src.w, src.h);
  nav.clear(0);

  if (r <= 0) {
    nav.occ = src.occ;
    return nav;
  }

  for (int y = 0; y < src.h; y++) {
    for (int x = 0; x < src.w; x++) {
      bool occ = false;
      for (int dy = -r; dy <= r && !occ; dy++) {
        for (int dx = -r; dx <= r && !occ; dx++) {
          int nx = x + dx, ny = y + dy;
          if (src.in_bounds(nx, ny) && src.is_occupied(nx, ny)) occ = true;
        }
      }
      nav.occ[y*nav.w + x] = occ ? 1 : 0;
    }
  }
  return nav;
}

static void set_cell(GridMap& map, int x, int y, int occ) {
  if (!map.in_bounds(x, y)) return;
  map.occ[y*map.w + x] = occ ? 1 : 0;
}

static void carve_rect_free(GridMap& map, int x0, int y0, int x1, int y1) {
  if (x0 > x1) std::swap(x0, x1);
  if (y0 > y1) std::swap(y0, y1);
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++)
      set_cell(map, x, y, 0);
}

static void clamp_inside_map(VehicleState& vs, const GridMap& map) {
  const double margin = 1.5;
  const double xmin = margin;
  const double ymin = margin;
  const double xmax = (double)map.w - margin;
  const double ymax = (double)map.h - margin;

  bool clamped = false;
  if (vs.x < xmin) { vs.x = xmin; clamped = true; }
  if (vs.y < ymin) { vs.y = ymin; clamped = true; }
  if (vs.x > xmax) { vs.x = xmax; clamped = true; }
  if (vs.y > ymax) { vs.y = ymax; clamped = true; }

  if (clamped) {
    vs.v = 0.0;
    vs.yaw = std::remainder(vs.yaw, 2.0 * 3.14159265358979323846);
  }
}

enum class DoorPhase { NONE, APPROACH, CROSS };

int main(int argc, char** argv) {
  const std::string mode = get_arg(argc, argv, "--mode", "static");
  const uint32_t seed = (uint32_t)get_arg_int(argc, argv, "--seed", 123);

  int steps = get_arg_int(argc, argv, "--steps", 1200);
  if (!has_key(argc, argv, "--steps") && mode == "replan_demo") steps = 2600;

  const double dt = get_arg_double(argc, argv, "--dt", 0.02);
  const int map_w = get_arg_int(argc, argv, "--map_w", 80);
  const int map_h = get_arg_int(argc, argv, "--map_h", 60);

  const double noise_pos = get_arg_double(argc, argv, "--noise_pos", 2.0);
  const double p_detect  = get_arg_double(argc, argv, "--p_detect", 1.0);

  const double gps_sigma = get_arg_double(argc, argv, "--gps_sigma", 0.5);
  const double gps_p_fix = get_arg_double(argc, argv, "--gps_p_fix", 1.0);

  const int lidar_beams = get_arg_int(argc, argv, "--lidar_beams", 31);
  const double lidar_fov_deg = get_arg_double(argc, argv, "--lidar_fov_deg", 90.0);
  const double lidar_range = get_arg_double(argc, argv, "--lidar_range", 12.0);
  const double lidar_step  = get_arg_double(argc, argv, "--lidar_step", 0.2);

  const double ahead_cone_deg = get_arg_double(argc, argv, "--ahead_cone_deg", 50.0);
  const double ahead_thresh   = get_arg_double(argc, argv, "--ahead_thresh", 4.0);

  const double target_vx = get_arg_double(argc, argv, "--target_vx", (mode=="moving") ? -1.5 : 0.0);
  const double target_vy = get_arg_double(argc, argv, "--target_vy", (mode=="moving") ?  0.5 : 0.0);
  const double lookahead = get_arg_double(argc, argv, "--lookahead", 3.5);
  const double max_speed = get_arg_double(argc, argv, "--max_speed", (mode=="replan_demo") ? 5.2 : 8.0);
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

  const int wx0 = map_w/2 - 1, wx1 = map_w/2 + 1;
  const int door_y0 = map_h/2 - 5;
  const int door_y1 = map_h/2 + 5;

  const Vec2 door_approach((double)(map_w/2 - 10), (double)(map_h/2));
  const Vec2 door_cross   ((double)(map_w/2 + 14), (double)(map_h/2));

  if (mode != "replan_demo") {
    map.add_rect_obstacle(10, 10, 14, 40);
    map.add_rect_obstacle(25,  0, 28, 32);
    map.add_rect_obstacle(40, 22, 60, 25);
    map.add_random_rect_obstacles(rng, 6, 3, 8, 3, 10);
  }

  // Vehicle
  VehicleState vs{};
  vs.x = 5.0; vs.y = 5.0; vs.yaw = 0.0; vs.v = 0.0;

  KinematicBicycleParams vehp{};
  vehp.wheelbase_L = 2.6;
  vehp.max_steer_rad = deg2rad(32.0);
  vehp.max_accel = 3.5;
  vehp.max_speed = max_speed;
  KinematicBicycleModel veh(vehp);

  // Target truth
  Vec2 target_p0((double)map_w - 8.0, (double)map_h - 8.0);
  Vec2 target_v(target_vx, target_vy);

  // Sensors
  Radar2D radar;
  radar.sigma_pos = noise_pos;
  radar.p_detect = clampd(p_detect, 0.0, 1.0);

  GPS2D gps;
  gps.sigma_pos = gps_sigma;
  gps.p_fix = clampd(gps_p_fix, 0.0, 1.0);

  Lidar2D lidar;
  lidar.beams = lidar_beams;
  lidar.fov_rad = deg2rad(lidar_fov_deg);
  lidar.max_range = lidar_range;
  lidar.step = lidar_step;

  // Tracking
  AlphaBetaAdapter tracker;
  tracker.impl.alpha = 0.85;
  tracker.impl.beta  = 0.005;
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
  ctrl_w.write_row({"t","steer_rad","accel","v_ref","cte","wp_idx"});

  CSVWriter evt_w((out_dir / "events.csv").string());
  evt_w.write_row({"t","event","detail"});

  CSVWriter est_w((out_dir / "target_est.csv").string());
  est_w.write_row({"t","det","zx","zy","xhat","yhat","vxhat","vyhat"});

  CSVWriter gps_w((out_dir / "gps.csv").string());
  gps_w.write_row({"t","fix","zx","zy","x_truth","y_truth"});

  CSVWriter lidar_w((out_dir / "lidar.csv").string());
  lidar_w.write_row({"t","min_range","ahead_blocked"});

  auto emit_event = [&](double tsec, const char* ev, const std::string& detail){
    evt_w.write_row({to_string_fixed(tsec, 4), ev, detail});
  };

  // Planning
  int plan_id = 0;
  std::vector<GridCell> path_cells;
  std::vector<Vec2> path_world;
  int wp_idx = 0;

  const int clearance_cells = (mode == "replan_demo") ? 0 : 1;
  GridMap nav_map = build_nav_map_inflated(map, clearance_cells);

  auto rebuild_nav = [&](){
    nav_map = build_nav_map_inflated(map, clearance_cells);
  };

  auto dump_plan = [&](){
    for (size_t i = 0; i < path_world.size(); i++) {
      plan_w.write_row({
        std::to_string(plan_id),
        std::to_string((int)i),
        to_string_fixed(path_world[i].x, 4),
        to_string_fixed(path_world[i].y, 4)
      });
    }
    plan_id++;
  };

  auto plan_now = [&](const Vec2& goal_in, const char* why)->bool {
    GridCell s = nav_map.world_to_cell(Vec2(vs.x, vs.y));
    GridCell g0 = nav_map.world_to_cell(goal_in);

    if (!nav_map.in_bounds(s.x, s.y)) {
      emit_event(0.0, "NO_PATH", std::string(why) + ":start_oob");
      return false;
    }

    GridCell g;
    if (!find_nearest_free_cell(nav_map, g0, 20, g)) {
      emit_event(0.0, "NO_PATH", std::string(why) + ":goal_blocked");
      return false;
    }

    AStarPlanner planner;
    planner.allow_diagonal = true;
    planner.corner_cutting = false;

    bool ok = planner.plan(nav_map, s, g, path_cells);
    if (!ok) {
      emit_event(0.0, "NO_PATH", std::string(why) + ":astar_fail");
      return false;
    }

    path_world.clear();
    path_world.reserve(path_cells.size());
    for (auto& c : path_cells) path_world.push_back(nav_map.cell_center_world(c));

    wp_idx = 0;
    dump_plan();
    return true;
  };

  const bool tracking_enabled = (mode == "moving");

  Vec2 final_goal = target_p0;
  Vec2 goal_world = final_goal;

  if (!plan_now(goal_world, "initial")) {
    std::fprintf(stderr, "NO_PATH\n");
    write_map_csv(out_dir, map);
    return 3;
  }

  const int inject_step = (mode=="replan_demo") ? (steps/3) : -1;

  GridCell last_goal_cell{-9999,-9999};
  double t = 0.0;

  bool in_collision_prev = false;
  int collision_cooldown = 0;

  DoorPhase door_phase = DoorPhase::NONE;

  int hb_cd = 0;

  for (int k = 0; k < steps; k++, t += dt) {
    if (mode == "replan_demo") clamp_inside_map(vs, map);

    if (hb_cd <= 0) {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "x=%.2f y=%.2f v=%.2f phase=%d wp=%d",
                    vs.x, vs.y, vs.v, (int)door_phase, wp_idx);
      emit_event(t, "STATUS", buf);
      hb_cd = (int)std::round(2.0 / dt);
    } else hb_cd--;

    // GPS
    Vec2 gps_z;
    bool gps_fix = gps.measure(rng, vs, gps_z);
    gps_w.write_row({
      to_string_fixed(t, 4),
      gps_fix ? "1" : "0",
      gps_fix ? to_string_fixed(gps_z.x, 4) : "nan",
      gps_fix ? to_string_fixed(gps_z.y, 4) : "nan",
      to_string_fixed(vs.x, 4),
      to_string_fixed(vs.y, 4)
    });

    // Lidar
    std::vector<double> ranges = lidar.scan(map, Vec2(vs.x, vs.y), vs.yaw);
    double minr = lidar.max_range;
    for (double r : ranges) if (r < minr) minr = r;

    bool ahead_blocked = lidar.obstacle_ahead(
      map, Vec2(vs.x, vs.y), vs.yaw,
      deg2rad(ahead_cone_deg),
      ahead_thresh
    );

    lidar_w.write_row({
      to_string_fixed(t, 4),
      to_string_fixed(minr, 4),
      ahead_blocked ? "1" : "0"
    });

    // Truth target
    TargetTruth tt = target_truth_at(t, target_p0, target_v);
    truth_w.write_row({
      to_string_fixed(t, 4),
      to_string_fixed(tt.pos.x, 4),
      to_string_fixed(tt.pos.y, 4),
      to_string_fixed(tt.vel.x, 4),
      to_string_fixed(tt.vel.y, 4)
    });

    // Tracking
    bool det = true;
    Vec2 z = tt.pos;
    if (tracking_enabled) {
      det = radar.measure(rng, tt.pos, z);
      if (det) tracker.update(z, dt);
    } else {
      tracker.reset(tt.pos, tt.vel);
      z = tt.pos;
      det = true;
    }

    est_w.write_row({
      to_string_fixed(t, 4),
      det ? "1" : "0",
      det ? to_string_fixed(z.x, 4) : "nan",
      det ? to_string_fixed(z.y, 4) : "nan",
      to_string_fixed(tracker.pos_hat().x, 4),
      to_string_fixed(tracker.pos_hat().y, 4),
      to_string_fixed(tracker.vel_hat().x, 4),
      to_string_fixed(tracker.vel_hat().y, 4)
    });

    // Inject wall+door (replan_demo)
    if (k == inject_step) {
      map.add_rect_obstacle(wx0, 5, wx1, door_y0 - 1);
      map.add_rect_obstacle(wx0, door_y1 + 1, wx1, map_h - 6);

      carve_rect_free(map, wx0 - 3, door_y0 - 3, wx1 + 3, door_y1 + 3);

      rebuild_nav();
      emit_event(t, "REPLAN", "inject_wall_door");

      // ---- CRITICAL: snap vehicle near approach point (deterministic demo) ----
      GridCell ac = map.world_to_cell(door_approach);
      GridCell freec;
      if (!find_nearest_free_cell(map, ac, 10, freec)) {
        emit_event(t, "NO_PATH", "inject:approach_not_free");
        break;
      }
      Vec2 snap = map.cell_center_world(freec);
      vs.x = snap.x;
      vs.y = snap.y;
      vs.yaw = 0.0;
      vs.v = 0.0;
      emit_event(t, "TELEPORT", "snap_to_approach");

      door_phase = DoorPhase::APPROACH;
      goal_world = door_approach;

      if (!plan_now(goal_world, "after_inject")) {
        emit_event(t, "NO_PATH", "after_inject");
        break;
      }
    }

    // Goal select
    if (mode == "replan_demo") {
      if (door_phase == DoorPhase::APPROACH) goal_world = door_approach;
      else if (door_phase == DoorPhase::CROSS) goal_world = door_cross;
      else goal_world = final_goal;
    } else if (tracking_enabled) {
      const double tlead = 1.2;
      goal_world = tracker.pos_hat() + tracker.vel_hat() * tlead;
    } else {
      goal_world = tt.pos;
    }

    // Door transitions
    if (mode == "replan_demo") {
      if (door_phase == DoorPhase::APPROACH) {
        if ((Vec2(vs.x, vs.y) - door_approach).norm() < 3.0) {
          emit_event(t, "REPLAN", "door_aligned");
          door_phase = DoorPhase::CROSS;
          goal_world = door_cross;
          (void)plan_now(goal_world, "door_cross");
        }
      } else if (door_phase == DoorPhase::CROSS) {
        if ((Vec2(vs.x, vs.y) - door_cross).norm() < 3.0) {
          emit_event(t, "REPLAN", "door_reached");
          door_phase = DoorPhase::NONE;
          goal_world = final_goal;
          (void)plan_now(goal_world, "final_goal");
        }
      }
    }

    // goal moved replans (moving mode only)
    GridCell gc = nav_map.world_to_cell(goal_world);
    if (tracking_enabled && (gc.x != last_goal_cell.x || gc.y != last_goal_cell.y)) {
      emit_event(t, "REPLAN", "goal_moved");
      (void)plan_now(goal_world, "goal_moved");
      last_goal_cell = gc;
    }

    // waypoint progress
    while (!path_world.empty() && wp_idx + 1 < (int)path_world.size()) {
      double d = (Vec2(vs.x, vs.y) - path_world[(size_t)wp_idx]).norm();
      if (d < 1.0) wp_idx++;
      else break;
    }

    std::vector<Vec2> subpath;
    if (!path_world.empty()) {
      int start = std::max(0, std::min(wp_idx, (int)path_world.size() - 1));
      subpath.assign(path_world.begin() + start, path_world.end());
    }

    if (subpath.empty()) emit_event(t, "STALL", "empty_path");

    // Control
    double cte = 0.0;
    double steer = pp.compute_steer(Vec2(vs.x, vs.y), vs.yaw, subpath, cte);
    if (!std::isfinite(steer)) steer = 0.0;
    steer = clampd(steer, -vehp.max_steer_rad, vehp.max_steer_rad);

    double dist_goal = (Vec2(vs.x, vs.y) - goal_world).norm();

    double v_ref = (dist_goal < 10.0) ? clampd(dist_goal * 0.7, 0.0, vehp.max_speed) : vehp.max_speed;
    if (mode == "replan_demo" && door_phase == DoorPhase::CROSS) v_ref = std::min(v_ref, 2.2);
    if (collision_cooldown > 0) { collision_cooldown--; v_ref = 0.0; }

    double accel_cmd = speed_pid.step(v_ref - vs.v, dt);
    accel_cmd = clampd(accel_cmd, -vehp.max_accel, vehp.max_accel);

    VehicleControl u{};
    u.steer_rad = steer;
    u.accel = accel_cmd;
    veh.step(vs, u, dt);

    // collision
    GridCell vc = map.world_to_cell(Vec2(vs.x, vs.y));
    bool in_collision = (!map.in_bounds(vc.x, vc.y)) || map.is_occupied(vc.x, vc.y);

    if (in_collision && !in_collision_prev) {
      emit_event(t, "COLLISION", "vehicle_in_occupied");
      emit_event(t, "REPLAN", "collision_recovery");
    }

    if (in_collision) {
      vs.v = 0.0;
      double back = 0.95;
      vs.x -= std::cos(vs.yaw) * back;
      vs.y -= std::sin(vs.yaw) * back;
      double kick = deg2rad(28.0) * ((k & 1) ? 1.0 : -1.0);
      vs.yaw += kick;
      collision_cooldown = (int)std::round(0.7 / dt);
      (void)plan_now(goal_world, "collision_replan");
    }

    in_collision_prev = in_collision;

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
      to_string_fixed(cte, 4),
      std::to_string(wp_idx)
    });

    if (dist_goal < 2.0 && door_phase == DoorPhase::NONE) {
      emit_event(t, "REACHED", "goal");
      break;
    }
  }

  write_map_csv(out_dir, map);

  uint64_t h = fnv1a64_init();
  const char* files[] = {
    "truth.csv","state.csv","plan.csv","control.csv","events.csv",
    "target_est.csv","map.csv","gps.csv","lidar.csv"
  };
  for (auto* fn : files) {
    uint64_t fh = hash_file_bytes(out_dir / fn);
    h = fnv1a64_update_u64(h, fh);
  }

  if (hash_on) std::fprintf(stderr, "FNV1A64=%016llx\n", (unsigned long long)h);
  return 0;
}