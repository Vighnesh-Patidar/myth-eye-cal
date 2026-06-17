#pragma once

// DeviceRegistry — passive discovery + a user-controlled connect allowlist.
//
// The transport hears beacons from nearby phones (data frames, presence
// announces, or replies to a scan probe). Every heard device is recorded here
// with its last-seen time, position and LOS state. CONNECTING is manual by
// default: in manual mode only devices the operator explicitly connects are
// fused into the local pose, so "who do I trust" stays in the user's hands.
// Switch to auto mode to fuse every visible device (the old behaviour).
//
// Transport-agnostic and header-only (zero deps) so it is shared by the UDP
// stopgap, the mith runtime, and the JNI bridge, and unit-tested on Linux.

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mec {

struct DiscoveredDevice {
    uint64_t id = 0;
    double   last_seen_s = 0.0;
    float    spx = 0.0f, spy = 0.0f, spz = 0.0f; // last reported world position
    uint8_t  los_state = 0;                       // last reported LOS (§3.1)
    bool     has_data = false;  // sent at least one real keypoint beacon
    bool     connected = false; // operator has connected to it
};

class DeviceRegistry {
public:
    explicit DeviceRegistry(double ttl_s = 8.0) : ttl_s_(ttl_s) {}

    // Record a device heard on the wire. has_data = it carried an actual
    // keypoint frame (vs. a presence/probe header only).
    void observe(uint64_t id, double now_s, float spx, float spy, float spz,
                 uint8_t los_state, bool has_data) {
        if (id == 0) return;
        DiscoveredDevice& d = seen_[id];
        d.id = id;
        d.last_seen_s = now_s;
        d.spx = spx; d.spy = spy; d.spz = spz;
        d.los_state = los_state;
        d.has_data = d.has_data || has_data;
    }

    void connect(uint64_t id)    { if (id) connected_.insert(id); }
    void disconnect(uint64_t id) { connected_.erase(id); }
    void disconnect_all()        { connected_.clear(); }

    // Connect every device currently visible (heard within ttl).
    void connect_all(double now_s) {
        for (const auto& kv : seen_)
            if (now_s - kv.second.last_seen_s <= ttl_s_) connected_.insert(kv.first);
    }

    bool manual() const     { return manual_; }
    void set_manual(bool m) { manual_ = m; }

    bool is_connected(uint64_t id) const { return connected_.count(id) != 0; }

    // Whether a received beacon from `id` should be fused into the local pose.
    bool should_fuse(uint64_t id) const {
        return !manual_ || connected_.count(id) != 0;
    }

    size_t connected_count() const { return connected_.size(); }

    // Snapshot of devices heard within ttl, most-recently-seen first. Prunes
    // stale entries (connections persist even if a device drops off and
    // reappears).
    std::vector<DiscoveredDevice> list(double now_s) {
        std::vector<DiscoveredDevice> out;
        for (auto it = seen_.begin(); it != seen_.end();) {
            if (now_s - it->second.last_seen_s > ttl_s_) { it = seen_.erase(it); continue; }
            DiscoveredDevice d = it->second;
            d.connected = connected_.count(d.id) != 0;
            out.push_back(d);
            ++it;
        }
        std::sort(out.begin(), out.end(), [](const DiscoveredDevice& a, const DiscoveredDevice& b) {
            return a.last_seen_s > b.last_seen_s;
        });
        return out;
    }

private:
    double ttl_s_;
    bool   manual_ = true; // default: operator picks who fuses
    std::unordered_map<uint64_t, DiscoveredDevice> seen_;
    std::unordered_set<uint64_t> connected_;
};

} // namespace mec
