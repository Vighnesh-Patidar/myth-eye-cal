#pragma once

// UdpBeaconTransport — stopgap multi-device transport for the user beacon
// channel (§4.5), used until the real mith-atomas transport is vendored
// (§15.6, §15.10). Each node broadcasts its 128-byte KeypointFramePayload (plus
// sender id, LOS state, and world position) over Wi-Fi UDP; every node receives
// its neighbours' beacons and fuses them. Portable POSIX sockets (Linux +
// Android NDK), non-blocking, no heap in the steady state beyond the poll list.
//
// Clock sync is NOT assumed: the receiver stamps arrival time itself (the
// aggregator's fusion window uses that), sidestepping cross-device clocks.

#include <array>
#include <cstdint>
#include <vector>

namespace mec {

inline constexpr uint32_t kUdpBeaconMagic = 0x4D454331; // "MEC1"

// One neighbour beacon as received off the wire (no mith dependency).
struct BeaconObservation {
    uint64_t sender = 0;
    uint8_t  los_state = 0;
    float    spx = 0.0f, spy = 0.0f, spz = 0.0f; // sender world position
    std::array<uint8_t, 128> payload{};
};

class UdpBeaconTransport {
public:
    UdpBeaconTransport() = default;
    ~UdpBeaconTransport();
    UdpBeaconTransport(const UdpBeaconTransport&) = delete;
    UdpBeaconTransport& operator=(const UdpBeaconTransport&) = delete;

    // Bind a UDP socket on listen_port and target `dest` for outgoing beacons.
    // dest defaults to the limited broadcast address; dest_port 0 = listen_port
    // (the shared beacon port). Returns false on failure.
    bool start(uint16_t listen_port, uint64_t self_node_id,
               const char* dest = "255.255.255.255", uint16_t dest_port = 0);
    void stop();

    // Send our beacon to the subnet (or configured dest).
    void broadcast(const std::array<uint8_t, 128>& payload, uint8_t los_state,
                   float spx, float spy, float spz);

    // Drain all pending received beacons (non-blocking). Our own packets
    // (matching self_node_id) are filtered out.
    std::vector<BeaconObservation> poll();

    uint16_t listen_port() const { return listen_port_; }
    bool running() const { return fd_ >= 0; }

private:
    int      fd_ = -1;
    uint16_t listen_port_ = 0;
    uint16_t dest_port_ = 0;
    uint64_t self_ = 0;
    uint32_t dest_addr_ = 0; // network byte order
};

} // namespace mec
