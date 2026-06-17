cmake_minimum_required(VERSION 3.16)
project(myth_eye_cal LANGUAGES CXX)

# --- Myth-Eye-Cal fusion core ---------------------------------------------
# Pure-geometry / pure-math fusion core plus the first-party fusion ECS
# (include/mec/ecs). Zero Android, zero OpenCV dependencies — testable on
# Linux x86 (see §13). Inter-device comms come from the real mith-atomas
# runtime (MEC_USE_MITH) or the UDP stopgap.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# Optional sanitizer build for the leak / UB gate (§13, CI "sanitize" job):
#   cmake -S . -B build-asan -DMEC_SANITIZE=address,undefined && ctest --test-dir build-asan
# Runs the whole test suite under AddressSanitizer + LeakSanitizer + UBSan.
set(MEC_SANITIZE "" CACHE STRING "Comma-separated -fsanitize list (e.g. address,undefined)")
if(MEC_SANITIZE)
  add_compile_options(-fsanitize=${MEC_SANITIZE} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${MEC_SANITIZE})
endif()

# Core library: fusion math, Kalman tracking, keypoint projection.
add_library(mec_core
  src/fusion/multi_observer_fusion.cpp
  src/fusion/keypoint_kalman.cpp
  src/observer/keypoint_projector.cpp
  src/observer/lucas_kanade.cpp
  src/observer/temporal_stereo_depth.cpp
  src/observer/imu_integrator.cpp
  src/render/websocket_render_server.cpp
  src/transport/udp_beacon_transport.cpp
)
target_include_directories(mec_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Simulated LOS node demo (no camera): generates synthetic observations,
# fuses them, and prints serialised pose frames (§6.3 format).
add_executable(sim_pose_demo examples/sim_pose_demo/main.cpp)
target_link_libraries(sim_pose_demo PRIVATE mec_core)

# Render server demo: fusion sim served live over WebSocket to the browser.
find_package(Threads REQUIRED)
add_executable(render_server_demo examples/render_server_demo/main.cpp)
target_link_libraries(render_server_demo PRIVATE mec_core Threads::Threads)

# ECS pipeline demo: §9 systems through the mec fusion scheduler.
add_executable(ecs_pipeline_demo examples/ecs_pipeline_demo/main.cpp)
target_link_libraries(ecs_pipeline_demo PRIVATE mec_core Threads::Threads)

# Host-side micro-benchmark + accuracy harness (docs/METRICS_REPORT.md).
add_executable(mec_bench tools/bench.cpp)
target_link_libraries(mec_bench PRIVATE mec_core)

# Real mith-atomas comms (§15.13). OFF by default; turn on once the submodule
# is present at third_party/mith-atomas:
#   cmake -S . -B build-mith -DMEC_USE_MITH=ON -DMITH_BUILD_TESTS=OFF -DMITH_BUILD_EXAMPLES=OFF
option(MEC_USE_MITH "Build the real mith-atomas integration (MithRuntime)" OFF)
if(MEC_USE_MITH)
    add_subdirectory(third_party/mith-atomas ${CMAKE_BINARY_DIR}/mith-atomas)
    add_executable(mith_node_demo
        examples/mith_node_demo/main.cpp
        src/transport/mith_runtime.cpp)
    target_link_libraries(mith_node_demo PRIVATE mec_core mith::mith Threads::Threads)
    target_compile_definitions(mith_node_demo PRIVATE MEC_USE_MITH=1)
endif()

# --- Tests -----------------------------------------------------------------
enable_testing()

function(mec_add_test name path)
  add_executable(${name} ${path})
  target_link_libraries(${name} PRIVATE mec_core)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

mec_add_test(test_multi_observer_fusion tests/unit/test_multi_observer_fusion.cpp)
mec_add_test(test_anisotropic_fusion    tests/unit/test_anisotropic_fusion.cpp)
mec_add_test(test_fusion_edges          tests/unit/test_fusion_edges.cpp)
mec_add_test(test_kalman_tracker        tests/unit/test_kalman_tracker.cpp)
mec_add_test(test_keypoint_projector    tests/unit/test_keypoint_projector.cpp)
mec_add_test(test_los_detector          tests/unit/test_los_detector.cpp)
mec_add_test(test_lucas_kanade          tests/unit/test_lucas_kanade.cpp)
mec_add_test(test_temporal_stereo       tests/unit/test_temporal_stereo.cpp)
mec_add_test(test_imu_integrator        tests/unit/test_imu_integrator.cpp)
mec_add_test(test_device_registry       tests/unit/test_device_registry.cpp)
mec_add_test(test_wire_payload          tests/unit/test_wire_payload.cpp)
mec_add_test(test_pose_serialiser       tests/unit/test_pose_serialiser.cpp)
mec_add_test(test_math                  tests/unit/test_math.cpp)
mec_add_test(test_ecs_world             tests/unit/test_ecs_world.cpp)
mec_add_test(test_fusion_pipeline       tests/integration/test_fusion_pipeline.cpp)
mec_add_test(test_websocket_server      tests/integration/test_websocket_server.cpp)
mec_add_test(test_websocket_edges       tests/integration/test_websocket_edges.cpp)
mec_add_test(test_udp_transport         tests/integration/test_udp_transport.cpp)
mec_add_test(test_udp_transport_edges   tests/integration/test_udp_transport_edges.cpp)

add_executable(test_ecs_pipeline tests/integration/test_ecs_pipeline.cpp)
target_link_libraries(test_ecs_pipeline PRIVATE mec_core)
add_test(NAME test_ecs_pipeline COMMAND test_ecs_pipeline)
