// Exhaustive tests for the §4.5 wire payload helpers in types.h: fixed-point
// metre packing (rounding + clamping), confidence quantisation, the 128-byte
// KeypointFramePayload layout, and a full pack -> bytes -> unpack round trip.

#include "mec/types.h"
#include "test_util.h"

#include <cstring>

using namespace mec;

static void test_pack_metres() {
    // Exact 1cm grid points round-trip.
    CHECK(pack_metres(0.0f) == 0);
    CHECK(pack_metres(1.0f) == 100);
    CHECK(pack_metres(-1.0f) == -100);
    CHECK_NEAR(unpack_metres(pack_metres(3.27f)), 3.27, 1e-6);
    CHECK_NEAR(unpack_metres(pack_metres(-3.27f)), -3.27, 1e-6);

    // Round-half-away-from-zero at the 1cm boundary.
    CHECK(pack_metres(0.005f) == 1);   // +0.5cm -> 1cm
    CHECK(pack_metres(0.004f) == 0);   // below half
    CHECK(pack_metres(-0.005f) == -1); // symmetric for negatives

    // Saturation at the int16 cm range (+-327.67m), no wraparound.
    CHECK(pack_metres(1000.0f) == 32767);
    CHECK(pack_metres(-1000.0f) == -32768);
    CHECK(pack_metres(327.67f) == 32767);
    CHECK(pack_metres(-327.68f) == -32768);

    // Sub-cm error bound across a representative sweep. The grid is 1cm so the
    // quantisation error is <= 0.5cm; allow a little float slack on top.
    for (float m = -50.0f; m <= 50.0f; m += 0.013f)
        CHECK(std::fabs(unpack_metres(pack_metres(m)) - m) <= 0.005f + 1e-3f);
}

static void test_pack_confidence() {
    CHECK(pack_confidence(-1.0f) == 0);
    CHECK(pack_confidence(0.0f) == 0);
    CHECK(pack_confidence(1.0f) == 255);
    CHECK(pack_confidence(2.0f) == 255); // clamp above 1
    CHECK(pack_confidence(0.5f) == 128); // 0.5*255+0.5 = 128
    // Monotonic & bounded round trip.
    float prev = -1.0f;
    for (int i = 0; i <= 100; ++i) {
        const float c = i / 100.0f;
        const float back = unpack_confidence(pack_confidence(c));
        CHECK(back >= 0.0f && back <= 1.0f);
        CHECK(back >= prev - 1e-6f); // non-decreasing
        prev = back;
    }
}

static void test_payload_layout() {
    static_assert(sizeof(KeypointFramePayload) == 128, "payload must be 128 bytes");
    CHECK(kMecSchemaId == 0x4D45); // "ME"
    CHECK(kNumKeypoints == 17);

    KeypointFramePayload pl;
    CHECK(pl.schema_id == kMecSchemaId); // default-constructed schema id
    pl.keypoint_count = kNumKeypoints;
    pl.frame_id = 65000;
    pl.timestamp_ms = 1234567u;
    for (int i = 0; i < kNumKeypoints; ++i) {
        pl.keypoints[i].wx = pack_metres(0.10f * i);
        pl.keypoints[i].wy = pack_metres(-0.05f * i);
        pl.keypoints[i].wz = pack_metres(1.0f + 0.01f * i);
        pl.keypoints[i].confidence = pack_confidence(i / 16.0f);
    }

    // Serialise into the 128-byte beacon slot and parse it back, as the
    // transport + aggregator do.
    std::array<uint8_t, 128> bytes{};
    std::memcpy(bytes.data(), &pl, sizeof(pl));
    KeypointFramePayload got;
    std::memcpy(&got, bytes.data(), sizeof(got));

    CHECK(got.schema_id == kMecSchemaId);
    CHECK(got.keypoint_count == kNumKeypoints);
    CHECK(got.frame_id == 65000);
    CHECK(got.timestamp_ms == 1234567u);
    for (int i = 0; i < kNumKeypoints; ++i) {
        CHECK_NEAR(unpack_metres(got.keypoints[i].wx), 0.10 * i, 0.005 + 1e-6);
        CHECK_NEAR(unpack_metres(got.keypoints[i].wz), 1.0 + 0.01 * i, 0.005 + 1e-6);
    }
}

int main() {
    test_pack_metres();
    test_pack_confidence();
    test_payload_layout();
    RUN_TESTS_RETURN();
}
