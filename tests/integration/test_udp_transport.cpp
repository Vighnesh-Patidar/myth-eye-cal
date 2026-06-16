// Loopback test for UdpBeaconTransport: two instances on localhost exchange a
// beacon (unicast to each other's port — the broadcast address is just a
// destination parameter in production).

#include "mec/transport/udp_beacon_transport.h"
#include "mec/types.h"
#include "../unit/test_util.h"

#include <array>
#include <cstring>
#include <unistd.h>

using namespace mec;

int main() {
    UdpBeaconTransport a, b;
    CHECK(a.start(42001, /*self=*/1, "127.0.0.1", 42002));
    CHECK(b.start(42002, /*self=*/2, "127.0.0.1", 42001));

    // Build a recognisable payload (valid schema id + a marker keypoint).
    std::array<uint8_t, 128> payload{};
    KeypointFramePayload pl;
    pl.keypoint_count = kNumKeypoints;
    pl.keypoints[0].wx = pack_metres(1.23f);
    pl.keypoints[0].confidence = pack_confidence(0.9f);
    std::memcpy(payload.data(), &pl, sizeof(pl));

    a.broadcast(payload, static_cast<uint8_t>(LOSState::TRACKING), 2.0f, 3.0f, 0.5f);

    // Poll B until the datagram arrives (loopback is fast; retry briefly).
    std::vector<BeaconObservation> rx;
    for (int i = 0; i < 200 && rx.empty(); ++i) {
        rx = b.poll();
        if (rx.empty()) usleep(1000);
    }
    CHECK(rx.size() == 1);
    if (!rx.empty()) {
        const BeaconObservation& o = rx[0];
        CHECK(o.sender == 1);
        CHECK(o.los_state == static_cast<uint8_t>(LOSState::TRACKING));
        CHECK_NEAR(o.spx, 2.0, 1e-4);
        CHECK_NEAR(o.spy, 3.0, 1e-4);
        CHECK_NEAR(o.spz, 0.5, 1e-4);
        KeypointFramePayload got;
        std::memcpy(&got, o.payload.data(), sizeof(got));
        CHECK(got.schema_id == kMecSchemaId);
        CHECK(got.keypoint_count == kNumKeypoints);
        CHECK_NEAR(unpack_metres(got.keypoints[0].wx), 1.23, 0.01);
    }

    // B's own poll must not surface A's packet as its own, and A shouldn't see
    // anything (A only sent; nothing was sent to A's port).
    CHECK(a.poll().empty());

    a.stop();
    b.stop();
    RUN_TESTS_RETURN();
}
