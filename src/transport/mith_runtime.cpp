// Real mith-atomas integration (compiled only when MEC_USE_MITH=ON).
// API per docs/MITH_INTEGRATION.md.

#include "mec/transport/mith_runtime.h"

#include "mith/comms/beacon_system.h"
#include "mith/comms/message.h"
#include "mith/comms/transport.h"
#include "mith/comms/udp_multicast_transport.h"
#include "mith/core/builtin_components.h"
#include "mith/core/world.h"

#include <cstring>

namespace mec {
namespace {

// Stable u64 id from a HierarchicalID's raw bytes (FNV-1a) for our neighbour
// bookkeeping. HierarchicalID is a trivially-copyable {SwarmID, UUID} value.
template <class IdT>
uint64_t hash_id(const IdT& id) {
    uint8_t buf[sizeof(IdT)];
    std::memcpy(buf, &id, sizeof(IdT));
    uint64_t h = 1469598103934665603ull;
    for (uint8_t b : buf) { h ^= b; h *= 1099511628211ull; }
    return h;
}

constexpr uint16_t kMecMsgType = mith::messages::CUSTOM + 0; // our keypoint frame tag

} // namespace

struct MithRuntime::Impl {
    std::unique_ptr<mith::World> world;
    uint32_t seq = 0;
};

MithRuntime::MithRuntime() : impl_(std::make_unique<Impl>()) {}
MithRuntime::~MithRuntime() = default; // World dtor closes sockets / joins threads

bool MithRuntime::start(uint16_t swarm_id, const char* group, uint16_t port,
                        const char* iface) {
    mith::WorldConfig cfg;
    cfg.swarm_id = mith::SwarmID{swarm_id};
    cfg.tick_rate_hz = 20.0f;
    cfg.beacon_rate_hz = 10.0f;
    cfg.scheduler_mode = mith::SchedulerMode::Sequential;

    mith::UDPMulticastTransport::Config tc;
    tc.group_address = group;
    tc.port = port;
    tc.interface_address = iface;
    tc.multicast_ttl = 1;
    tc.recv_buffer_bytes = 64 * 1024;

    impl_->world = std::make_unique<mith::World>(
        std::move(cfg), mith::UDPMulticastTransport::open(tc));
    impl_->world->register_system(std::make_unique<mith::BeaconSystem>(*impl_->world));
    impl_->world->register_system(std::make_unique<mith::DiscoverySystem>(*impl_->world));
    impl_->world->register_system(std::make_unique<mith::ClockSyncSystem>(*impl_->world));
    impl_->world->init();
    return true;
}

void MithRuntime::broadcast(const std::array<uint8_t, 128>& payload, uint8_t los_state,
                            float spx, float spy, float spz) {
    if (!impl_->world) return;
    auto& reg = impl_->world->registry();
    const auto self = impl_->world->self_id();

    // Replicated state (rides the auto beacon).
    auto& pos = reg.get<mith::PositionComponent>(self);
    pos.x = spx; pos.y = spy; pos.z = spz;
    reg.get<mith::BehaviourStateComponent>(self).state = los_state;

    // App payload (keypoint frame) rides a CUSTOM Message.
    mith::Message m;
    m.sender = impl_->world->identity();
    m.recipient = mith::BROADCAST_ID;
    m.type = kMecMsgType;
    m.seq = impl_->seq++;
    m.timestamp_s = impl_->world->synced_time_s();
    std::memcpy(m.payload.data(), payload.data(), 128);
    impl_->world->message_transport()->send_message(m);
}

std::vector<BeaconObservation> MithRuntime::poll() {
    std::vector<BeaconObservation> out;
    if (!impl_->world) return out;

    auto& cb = impl_->world->registry().get<mith::CommBufferComponent>(impl_->world->self_id());
    while (auto msg = cb.queue.pop()) {
        if (msg->type != kMecMsgType) continue;
        BeaconObservation o;
        o.sender = hash_id(msg->sender);
        o.los_state = 2; // default = mec::LOSState::TRACKING until paired below
        // Pair the payload with the sender's replicated neighbour state.
        const auto& nt = impl_->world->neighbour_table();
        for (auto it = nt.begin(); it != nt.end(); ++it) {
            const auto& e = *it;
            if (e.hid == msg->sender) {
                o.los_state = static_cast<uint8_t>(e.state.state);
                o.spx = e.position.x; o.spy = e.position.y; o.spz = e.position.z;
                break;
            }
        }
        std::memcpy(o.payload.data(), msg->payload.data(), 128);
        out.push_back(o);
    }
    return out;
}

void MithRuntime::tick() { if (impl_->world) impl_->world->tick(); }

double MithRuntime::synced_time_s() const {
    return impl_->world ? static_cast<double>(impl_->world->synced_time_s()) : 0.0;
}

size_t MithRuntime::neighbour_count() const {
    if (!impl_->world) return 0;
    size_t n = 0;
    const auto& nt = impl_->world->neighbour_table();
    for (auto it = nt.begin(); it != nt.end(); ++it) ++n;
    return n;
}

bool MithRuntime::running() const { return impl_->world != nullptr; }

} // namespace mec
