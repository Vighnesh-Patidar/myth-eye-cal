#include "mec/observer/lucas_kanade.h"
#include "test_util.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace mec;

// Smooth, gradient-rich analytic pattern so LK has texture to lock onto.
static float pattern(float x, float y) {
    float v = 128.0f + 50.0f * std::sin(0.20f * x)
                     + 40.0f * std::cos(0.13f * y)
                     + 30.0f * std::sin(0.07f * (x + y));
    return std::clamp(v, 0.0f, 255.0f);
}

// Fill an image with the pattern shifted by (dx,dy): img(p) = pattern(p - d).
static void fill(std::vector<uint8_t>& img, int w, int h, float dx, float dy) {
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            img[y * w + x] = static_cast<uint8_t>(pattern(x - dx, y - dy) + 0.5f);
}

int main() {
    const int w = 160, h = 120;
    std::vector<uint8_t> curr(w * h), prev(w * h);
    fill(curr, w, h, 0.0f, 0.0f);

    LucasKanade lk(w, h);

    // Tracking curr -> prev where prev(p) = pattern(p - d) yields flow = d.
    struct Case { float dx, dy; } cases[] = {{4, 0}, {0, 3}, {2.5f, -2.0f}, {6, 1}};
    const float pts[] = {80, 60, 50, 40, 100, 70};

    for (const Case& c : cases) {
        fill(prev, w, h, c.dx, c.dy);
        float flow[6];
        uint8_t st[3];
        lk.track(curr.data(), prev.data(), w, h, pts, 3, flow, st);
        for (int i = 0; i < 3; ++i) {
            CHECK(st[i] == 1);
            CHECK_NEAR(flow[2 * i], c.dx, 0.3);
            CHECK_NEAR(flow[2 * i + 1], c.dy, 0.3);
        }
    }

    RUN_TESTS_RETURN();
}
