// Edge-case coverage for UdpBeaconTransport beyond the happy-path handshake in
// test_udp_transport.cpp:
//   - an unstarted transport is inert (running()/broadcast()/poll() no-ops)
//   - packets with a foreign magic are dropped
//   - short/truncated datagrams are dropped
//   - a packet whose sender == self is filtered (own multicast echo)
//   - a well-formed foreign packet surfaces with the right kind + payload

#include "mec/transport/udp_beacon_transport.h"
#include "mec/types.h"
#include "../unit/test_util.h"

#include <cstring>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace mec;

// Mirror of the on-wire packet (must match udp_beacon_transport.cpp).
#pragma pack(push, 1)
struct WirePacket {
    uint32_t magic;
    uint64_t sender;
    uint8_t  kind;
    uint8_t  los_state;
    float    spx, spy, spz;
    uint8_t  payload[128];
};
#pragma pack(pop)
static_assert(sizeof(WirePacket) == 154, "wire packet layout drift");

static void send_raw(uint16_t port, const void* data, size_t n) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
    ::sendto(fd, data, n, 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    ::close(fd);
}

static std::vector<BeaconObservation> drain(UdpBeaconTransport& t) {
    std::vector<BeaconObservation> all, batch;
    for (int i = 0; i < 50; ++i) {
        batch = t.poll();
        for (auto& o : batch) all.push_back(o);
        usleep(1000);
        if (!batch.empty()) { // keep draining a touch in case of more
            for (int j = 0; j < 5; ++j) { batch = t.poll(); for (auto& o : batch) all.push_back(o); }
            break;
        }
    }
    return all;
}

static void test_unstarted_is_inert() {
    UdpBeaconTransport t;
    CHECK(!t.running());
    std::array<uint8_t, 128> payload{};
    t.broadcast(payload, 2, 0, 0, 0); // must not crash
    t.scan();
    t.announce_presence(2, 0, 0, 0);
    CHECK(t.poll().empty());
    CHECK(t.listen_port() == 0);
}

static void test_filtering() {
    const uint16_t port = 42071;
    const uint64_t self = 100;
    UdpBeaconTransport rx;
    CHECK(rx.start(port, self, "127.0.0.1", port));
    CHECK(rx.running());
    CHECK(rx.listen_port() == port);

    auto make = [&](uint32_t magic, uint64_t sender, uint8_t kind) {
        WirePacket p{};
        p.magic = magic;
        p.sender = sender;
        p.kind = kind;
        p.los_state = static_cast<uint8_t>(LOSState::TRACKING);
        p.spx = 1.0f; p.spy = 2.0f; p.spz = 3.0f;
        KeypointFramePayload pl;
        pl.keypoint_count = kNumKeypoints;
        pl.keypoints[0].wx = pack_metres(4.56f);
        std::memcpy(p.payload, &pl, sizeof(pl));
        return p;
    };

    // Wrong magic -> dropped.
    { WirePacket p = make(0xDEADBEEF, 7, kBeaconData); send_raw(port, &p, sizeof(p)); }
    // Truncated datagram -> dropped.
    { WirePacket p = make(kUdpBeaconMagic, 7, kBeaconData); send_raw(port, &p, 16); }
    // Own sender id -> filtered as the local echo.
    { WirePacket p = make(kUdpBeaconMagic, self, kBeaconData); send_raw(port, &p, sizeof(p)); }
    usleep(5000);
    CHECK(rx.poll().empty()); // none of the above should surface

    // A valid foreign data packet surfaces with the right metadata + payload.
    { WirePacket p = make(kUdpBeaconMagic, 42, kBeaconData); send_raw(port, &p, sizeof(p)); }
    auto got = drain(rx);
    CHECK(got.size() == 1);
    if (!got.empty()) {
        CHECK(got[0].sender == 42);
        CHECK(got[0].kind == kBeaconData);
        CHECK(got[0].los_state == static_cast<uint8_t>(LOSState::TRACKING));
        CHECK_NEAR(got[0].spx, 1.0, 1e-4);
        KeypointFramePayload pl;
        std::memcpy(&pl, got[0].payload.data(), sizeof(pl));
        CHECK(pl.schema_id == kMecSchemaId);
        CHECK_NEAR(unpack_metres(pl.keypoints[0].wx), 4.56, 0.01);
    }

    // A presence packet (no payload) surfaces with kind kBeaconPresence.
    { WirePacket p = make(kUdpBeaconMagic, 43, kBeaconPresence); send_raw(port, &p, sizeof(p)); }
    auto pres = drain(rx);
    CHECK(pres.size() == 1);
    if (!pres.empty()) CHECK(pres[0].kind == kBeaconPresence);

    rx.stop();
    CHECK(!rx.running());
}

int main() {
    test_unstarted_is_inert();
    test_filtering();
    RUN_TESTS_RETURN();
}
