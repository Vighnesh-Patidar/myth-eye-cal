#include "mec/observer/lucas_kanade.h"

#include <algorithm>

namespace mec {
namespace {

inline float sample(const std::vector<float>& img, int w, int h, float x, float y) {
    x = std::clamp(x, 0.0f, static_cast<float>(w - 1));
    y = std::clamp(y, 0.0f, static_cast<float>(h - 1));
    const int x0 = static_cast<int>(x);
    const int y0 = static_cast<int>(y);
    const int x1 = std::min(x0 + 1, w - 1);
    const int y1 = std::min(y0 + 1, h - 1);
    const float fx = x - x0, fy = y - y0;
    const float v00 = img[y0 * w + x0], v01 = img[y0 * w + x1];
    const float v10 = img[y1 * w + x0], v11 = img[y1 * w + x1];
    return (v00 * (1 - fx) + v01 * fx) * (1 - fy) +
           (v10 * (1 - fx) + v11 * fx) * fy;
}

} // namespace

LucasKanade::LucasKanade(int max_w, int max_h, int levels,
                         int window_radius, int max_iters)
    : levels_(std::max(1, levels)),
      win_(std::max(1, window_radius)),
      iters_(std::max(1, max_iters)) {
    pa_.resize(levels_);
    pb_.resize(levels_);
    int w = max_w, h = max_h;
    for (int l = 0; l < levels_; ++l) {
        const int cap = std::max(1, w) * std::max(1, h);
        pa_[l].img.resize(cap);
        pb_[l].img.resize(cap);
        w = std::max(1, w / 2);
        h = std::max(1, h / 2);
    }
}

void LucasKanade::build_pyramid(const uint8_t* src, int w, int h,
                                std::vector<Level>& pyr) {
    Level& L0 = pyr[0];
    L0.w = w; L0.h = h;
    if (static_cast<int>(L0.img.size()) < w * h) L0.img.resize(w * h);
    for (int i = 0; i < w * h; ++i) L0.img[i] = static_cast<float>(src[i]);

    for (int l = 1; l < levels_; ++l) {
        const int pw = pyr[l - 1].w, ph = pyr[l - 1].h;
        const int nw = std::max(1, pw / 2), nh = std::max(1, ph / 2);
        Level& L = pyr[l];
        L.w = nw; L.h = nh;
        if (static_cast<int>(L.img.size()) < nw * nh) L.img.resize(nw * nh);
        const std::vector<float>& P = pyr[l - 1].img;
        for (int y = 0; y < nh; ++y) {
            for (int x = 0; x < nw; ++x) {
                const int x0 = 2 * x, y0 = 2 * y;
                const int x1 = std::min(x0 + 1, pw - 1), y1 = std::min(y0 + 1, ph - 1);
                L.img[y * nw + x] = 0.25f * (P[y0 * pw + x0] + P[y0 * pw + x1] +
                                             P[y1 * pw + x0] + P[y1 * pw + x1]);
            }
        }
    }
}

void LucasKanade::track(const uint8_t* A, const uint8_t* B, int w, int h,
                        const float* points, int n,
                        float* flow_out, uint8_t* status_out) {
    build_pyramid(A, w, h, pa_);
    build_pyramid(B, w, h, pb_);

    for (int i = 0; i < n; ++i) {
        float gx = 0.0f, gy = 0.0f; // flow estimate at current level's scale
        bool ok = true;

        for (int l = levels_ - 1; l >= 0; --l) {
            const Level& LA = pa_[l];
            const Level& LB = pb_[l];
            const float scale = static_cast<float>(1 << l);
            const float px = points[2 * i] / scale;
            const float py = points[2 * i + 1] / scale;

            for (int it = 0; it < iters_; ++it) {
                float G00 = 0, G01 = 0, G11 = 0, b0 = 0, b1 = 0;
                for (int wy = -win_; wy <= win_; ++wy) {
                    for (int wx = -win_; wx <= win_; ++wx) {
                        const float ax = px + wx, ay = py + wy;
                        const float Ix = 0.5f * (sample(LA.img, LA.w, LA.h, ax + 1, ay) -
                                                 sample(LA.img, LA.w, LA.h, ax - 1, ay));
                        const float Iy = 0.5f * (sample(LA.img, LA.w, LA.h, ax, ay + 1) -
                                                 sample(LA.img, LA.w, LA.h, ax, ay - 1));
                        const float dI = sample(LA.img, LA.w, LA.h, ax, ay) -
                                         sample(LB.img, LB.w, LB.h, ax + gx, ay + gy);
                        G00 += Ix * Ix; G01 += Ix * Iy; G11 += Iy * Iy;
                        b0  += Ix * dI; b1  += Iy * dI;
                    }
                }
                const float det = G00 * G11 - G01 * G01;
                if (det <= 1e-6f) { ok = false; break; }
                const float dx = (G11 * b0 - G01 * b1) / det;
                const float dy = (G00 * b1 - G01 * b0) / det;
                gx += dx; gy += dy;
                if (dx * dx + dy * dy < 1e-4f) break;
                if (px + gx < -2 || px + gx > LA.w + 1 ||
                    py + gy < -2 || py + gy > LA.h + 1) { ok = false; break; }
            }
            if (!ok) break;
            if (l > 0) { gx *= 2.0f; gy *= 2.0f; } // propagate to finer level
        }

        flow_out[2 * i] = gx;
        flow_out[2 * i + 1] = gy;
        if (status_out) status_out[i] = ok ? 1 : 0;
    }
}

} // namespace mec
