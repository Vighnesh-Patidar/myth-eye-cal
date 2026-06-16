// mith_node_demo — exercises MithRuntime against the real mith-atomas runtime
// (compile/link/run check; run two instances to see them discover each other).
//   build: cmake -S . -B build-mith -DMEC_USE_MITH=ON && cmake --build build-mith
//   run:   ./build-mith/mith_node_demo   (start two for a 2-node comms test)

#include "mec/transport/mith_runtime.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

int main(int argc, char** argv) {
    const uint16_t swarm = 42;
    const int frames = (argc > 1) ? std::atoi(argv[1]) : 40;

    mec::MithRuntime rt;
    if (!rt.start(swarm, "239.10.20.30", 47474)) {
        std::fprintf(stderr, "MithRuntime.start() failed\n");
        return 1;
    }
    std::printf("mith node up (swarm %u) — broadcasting %d frames\n", swarm, frames);

    std::array<uint8_t, 128> payload{};
    payload[0] = 0x45; payload[1] = 0x4D; // "ME"-ish marker

    for (int i = 0; i < frames; ++i) {
        rt.broadcast(payload, /*los=TRACKING*/ 2, /*x*/ 1.0f, /*y*/ 0.0f, /*z*/ 1.5f);
        rt.tick();
        const auto rx = rt.poll();
        std::printf("tick %2d: neighbours=%zu received=%zu synced=%.2f\n",
                    i, rt.neighbour_count(), rx.size(), rt.synced_time_s());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::printf("done\n");
    return 0;
}
