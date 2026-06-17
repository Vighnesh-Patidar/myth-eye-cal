# NDK build of the Myth-Eye-Cal JNI bridge. Compiles the tested, portable
# mec_core sources (zero Android deps) plus the JNI glue into one .so. The
# fusion ECS is first-party (include/mec/ecs); inter-device comms come from the
# real mith-atomas submodule (MEC_USE_MITH) or the UDP stopgap.

cmake_minimum_required(VERSION 3.22.1)
project(mec_jni LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# android/app/src/main/cpp -> repository root is five levels up.
set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)

add_library(mec_jni SHARED
    mec_jni.cpp
    ${REPO}/src/fusion/multi_observer_fusion.cpp
    ${REPO}/src/fusion/keypoint_kalman.cpp
    ${REPO}/src/observer/keypoint_projector.cpp
    ${REPO}/src/observer/lucas_kanade.cpp
    ${REPO}/src/observer/temporal_stereo_depth.cpp
    ${REPO}/src/observer/imu_integrator.cpp
    ${REPO}/src/render/websocket_render_server.cpp
    ${REPO}/src/transport/udp_beacon_transport.cpp
)

target_include_directories(mec_jni PRIVATE
    ${REPO}/include
)

find_library(log-lib log)
target_link_libraries(mec_jni ${log-lib})

# Real mith-atomas comms (§15.13). Off by default (UDP stopgap is used).
# Turn on with -DMEC_USE_MITH=ON once the submodule is at third_party/mith-atomas.
option(MEC_USE_MITH "Use real mith-atomas for the beacon channel" OFF)
if(MEC_USE_MITH)
    add_subdirectory(${REPO}/third_party/mith-atomas ${CMAKE_BINARY_DIR}/mith-atomas)
    target_sources(mec_jni PRIVATE ${REPO}/src/transport/mith_runtime.cpp)
    target_link_libraries(mec_jni mith::mith)
    target_compile_definitions(mec_jni PRIVATE MEC_USE_MITH=1)
endif()
