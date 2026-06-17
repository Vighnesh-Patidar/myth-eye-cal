#pragma once

// MithRuntime — real mith-atomas backing for the user beacon channel (§15.10,
// docs/MITH_INTEGRATION.md). Drop-in for UdpBeaconTransport: same broadcast()
// shape and poll() -> BeaconObservation, so the fusion pipeline is unchanged.
//
// Uses BOTH mith channels as intended:
//   - the auto BeaconSystem carries this node's Position + BehaviourState (LOS),
//   - our 128-byte keypoint frame rides a Message (mith::messages::CUSTOM),
//   - poll() drains received CUSTOM payloads and pairs each with its sender's
//     neighbour-table entry (position + LOS) to build a BeaconObservation.
//
// Compiled only when MEC_USE_MITH=ON (needs the mith-atomas submodule). mith
// headers stay in the .cpp via pimpl. Drive it single-threaded from the same
// loop that mutates components (mith's EntityRegistry is not thread-safe).

#include "mec/transport/udp_beacon_transport.h" // BeaconObservation

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace mec {

class MithRuntime {
public:
    MithRuntime();
    ~MithRuntime();
    MithRuntime(const MithRuntime&) = delete;
    MithRuntime& operator=(const MithRuntime&) = delete;

    // Build the World on a UDP multicast transport + BeaconSystem and init().
    bool start(uint16_t swarm_id, const char* group, uint16_t port,
               const char* iface = "0.0.0.0");

    // Same signature as UdpBeaconTransport::broadcast: stamp this node's
    // Position + BehaviourState (replicated by the beacon) and send the payload
    // as a CUSTOM Message.
    void broadcast(const std::array<uint8_t, 128>& payload, uint8_t los_state,
                   float spx, float spy, float spz);

    // Drain received keypoint payloads, paired with sender neighbour state.
    std::vector<BeaconObservation> poll();

    // Discovery. scan() broadcasts a discovery message (mith already auto-
    // announces presence via its beacon; this is the explicit "request header").
    // neighbours() returns the current neighbour-table presence (every node
    // mith has discovered), as kBeaconPresence observations with no payload.
    void scan();
    std::vector<BeaconObservation> neighbours() const;

    void   tick();              // world.tick() — drives comms (call once per frame)
    double synced_time_s() const;
    size_t neighbour_count() const;
    bool   running() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace mec
