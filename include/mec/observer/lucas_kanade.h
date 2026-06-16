#pragma once

// LucasKanade — pyramidal sparse Lucas-Kanade optical flow (ARCHITECTURE.md
// §4.3). Portable C++17 reference implementation, no OpenCV. Pyramid buffers
// are pre-allocated in the constructor; track() performs no heap allocation in
// the steady state (hot-path constraint, §13).
//
// NOTE (§15.7): the doc specifies NEON intrinsics. This reference is portable
// scalar so it builds and is tested on Linux x86; a NEON-specialised inner
// loop is a build-time optimisation that does not change this interface.

#include <cstdint>
#include <vector>

namespace mec {

class LucasKanade {
public:
    LucasKanade(int max_w, int max_h, int levels = 3,
                int window_radius = 3, int max_iters = 20);

    // Track points from image A to image B (both Y planes, width*height bytes).
    // Solves for flow such that A(p) ~= B(p + flow). points / flow_out are
    // interleaved (x0,y0,x1,y1,...) in pixels. status_out[i] is 1 if the point
    // was tracked, 0 if the solve was degenerate or went out of bounds.
    void track(const uint8_t* A, const uint8_t* B, int w, int h,
               const float* points, int n,
               float* flow_out, uint8_t* status_out = nullptr);

    int levels() const { return levels_; }

private:
    struct Level {
        std::vector<float> img;
        int w = 0, h = 0;
    };
    void build_pyramid(const uint8_t* src, int w, int h, std::vector<Level>& pyr);

    int levels_, win_, iters_;
    std::vector<Level> pa_, pb_;
};

} // namespace mec
