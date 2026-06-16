# NDK build of the Myth-Eye-Cal JNI bridge. Compiles the tested, portable
# mec_core sources (zero Android deps) plus the JNI glue into one .so.
#
# NOTE: `mith::` is still the header-only MOCK (../../../../../mock). Swap that
# include path for the real mith-atomas submodule when it is vendored (§15.6).

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
    ${REPO}/mock            # mock MithAtomas — replace with the real submodule
)

find_library(log-lib log)
target_link_libraries(mec_jni ${log-lib})
