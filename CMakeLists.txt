cmake_minimum_required(VERSION 3.16)
project(myth_eye_cal LANGUAGES CXX)

# --- Myth-Eye-Cal fusion core ---------------------------------------------
# v0.1 scope: the pure-geometry / pure-math fusion core. Zero Android, zero
# MithAtomas, zero OpenCV dependencies — testable on Linux x86 (see §13).
# MithAtomas ECS systems (§9) are deferred until the submodule is vendored.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# Core library: fusion math, Kalman tracking, keypoint projection.
add_library(mec_core
  src/fusion/multi_observer_fusion.cpp
  src/fusion/keypoint_kalman.cpp
  src/observer/keypoint_projector.cpp
  src/render/websocket_render_server.cpp
)
target_include_directories(mec_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Simulated LOS node demo (no camera): generates synthetic observations,
# fuses them, and prints serialised pose frames (§6.3 format).
add_executable(sim_pose_demo examples/sim_pose_demo/main.cpp)
target_link_libraries(sim_pose_demo PRIVATE mec_core)

# Render server demo: fusion sim served live over WebSocket to the browser.
add_executable(render_server_demo examples/render_server_demo/main.cpp)
target_link_libraries(render_server_demo PRIVATE mec_core)
find_package(Threads REQUIRED)
target_link_libraries(render_server_demo PRIVATE Threads::Threads)

# --- Tests -----------------------------------------------------------------
enable_testing()

function(mec_add_test name path)
  add_executable(${name} ${path})
  target_link_libraries(${name} PRIVATE mec_core)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

mec_add_test(test_multi_observer_fusion tests/unit/test_multi_observer_fusion.cpp)
mec_add_test(test_kalman_tracker        tests/unit/test_kalman_tracker.cpp)
mec_add_test(test_keypoint_projector    tests/unit/test_keypoint_projector.cpp)
mec_add_test(test_fusion_pipeline       tests/integration/test_fusion_pipeline.cpp)
mec_add_test(test_websocket_server      tests/integration/test_websocket_server.cpp)
