cmake_minimum_required(VERSION 3.16)
project(autonomous_ground_vehicle_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(agv_sim
  src/main.cpp
  src/sim/config.h
  src/sim/rng.h
  src/sim/fnv1a.h
  src/sim/csv.h
  src/sim/math/vec2.h

  src/sim/world/grid_map.h
  src/sim/world/grid_map.cpp

  src/sim/vehicle/vehicle_state.h
  src/sim/vehicle/kinematic_bicycle.h
  src/sim/vehicle/kinematic_bicycle.cpp

  src/sim/planning/astar.h
  src/sim/planning/astar.cpp

  src/sim/control/pid.h
  src/sim/control/pure_pursuit.h
  src/sim/control/pure_pursuit.cpp

  src/sim/tracking/tracker_iface.h
  src/sim/tracking/alpha_beta_tracker.h
  src/sim/tracking/alpha_beta_tracker.cpp

  src/sim/sensors/radar2d.h
  src/sim/sensors/radar2d.cpp
  src/sim/sensors/gps.h
  src/sim/sensors/gps.cpp
  src/sim/sensors/lidar2d.h
  src/sim/sensors/lidar2d.cpp
)

if (MSVC)
  target_compile_options(agv_sim PRIVATE /W4 /permissive-)
else()
  target_compile_options(agv_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()