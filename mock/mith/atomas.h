#pragma once

// ============================================================================
// MOCK MithAtomas runtime — a stand-in for the real `mith-atomas` submodule
// (ARCHITECTURE.md §2, §8, §9, §14). It provides just enough of the surface
// the Myth-Eye-Cal ECS systems need — Entity/Component store, SystemScheduler,
// NeighbourTable, UserNeighbourTable, the user beacon channel, and the core
// StateVector components — so the §9 systems can be implemented and tested on
// Linux before the coordination layer exists.
//
// NOT a real implementation: no transport, clock sync, Ed25519 auth, partition
// merge, or fault recovery. Swap this directory out for the real submodule
// (same `mith::` names) when it lands.
// ============================================================================

#include <algorithm>
#include <any>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace mith {

using EntityId = uint32_t;
using NodeId   = uint64_t;
inline constexpr EntityId kInvalidEntity = 0;

// CRTP component category tags (real MithAtomas separates hot/cold storage;
// the mock stores both identically and only uses the tag for documentation).
template <class Derived> struct HotComponent {};
template <class Derived> struct ColdComponent {};

// --- Core StateVector components (replicated by the coordination layer) ----
struct PositionComponent    { float x = 0, y = 0, z = 0; };               // GPS / manual pin
struct OrientationComponent { float qw = 1, qx = 0, qy = 0, qz = 0; };    // IMU quaternion
struct BehaviourStateComponent { uint8_t los_state = 0; };                // §3.3 carries LOS

// --- Neighbour discovery (§2 NeighbourTable) -------------------------------
struct Neighbour {
    NodeId                 id = 0;
    BehaviourStateComponent behaviour{};
    double                 last_seen_s = 0.0;
};
struct NeighbourTable { std::vector<Neighbour> neighbours; };

// --- User beacon channel (§4.5): 128-byte payloads to/from the swarm -------
struct UserStateVector {
    NodeId   sender = 0;
    uint8_t  los_state = 0;   // sender's LOS, mirrored from its BehaviourState
    double   recv_time_s = 0.0;
    // Sender's world position, from its replicated PositionComponent. Lets the
    // receiver reconstruct each observation's view ray for anisotropic fusion
    // (§15.3). Zero = unknown.
    float    spx = 0.0f, spy = 0.0f, spz = 0.0f;
    std::array<uint8_t, 128> payload{};
};
struct UserNeighbourTable {
    std::vector<UserStateVector> entries;
    void clear() { entries.clear(); }
};
struct UserBeaconChannel {
    std::array<uint8_t, 128> last_payload{};
    bool     sent = false;
    uint64_t count = 0;
    std::function<void(const std::array<uint8_t, 128>&)> on_broadcast; // test hook
    void broadcast(const std::array<uint8_t, 128>& p) {
        last_payload = p; sent = true; ++count;
        if (on_broadcast) on_broadcast(p);
    }
};

// --- Entity / component store ----------------------------------------------
class World {
public:
    EntityId create_entity() { return next_++; }
    EntityId local() const { return local_; }
    void     set_local(EntityId e) { local_ = e; }

    double now_s() const { return now_; }
    void   set_now(double t) { now_ = t; }

    template <class C> C& add(EntityId e, C value = C{}) {
        auto& store = stores_[std::type_index(typeid(C))];
        auto it = store.find(e);
        if (it == store.end()) it = store.emplace(e, std::any(std::move(value))).first;
        else it->second = std::any(std::move(value));
        return *std::any_cast<C>(&it->second);
    }
    template <class C> C* get(EntityId e) {
        auto si = stores_.find(std::type_index(typeid(C)));
        if (si == stores_.end()) return nullptr;
        auto it = si->second.find(e);
        return (it == si->second.end()) ? nullptr : std::any_cast<C>(&it->second);
    }
    template <class C> bool has(EntityId e) { return get<C>(e) != nullptr; }
    template <class C> C& get_or_add(EntityId e) {
        C* p = get<C>(e);
        return p ? *p : add<C>(e);
    }

    NeighbourTable     neighbours;
    UserNeighbourTable user_neighbours;
    UserBeaconChannel  user_beacon;

private:
    EntityId next_  = 1;
    EntityId local_ = kInvalidEntity;
    double   now_   = 0.0;
    std::unordered_map<std::type_index, std::unordered_map<EntityId, std::any>> stores_;
};

// --- Systems & scheduler (§9) ----------------------------------------------
class System {
public:
    virtual ~System() = default;
    virtual const char* name() const = 0;
    virtual double      rate_hz() const = 0;          // target execution rate
    virtual void        update(World& w, double dt) = 0;
};

// Mixed-rate scheduler. Each system runs at ~rate_hz; `after` names enforce
// execution order within a tick (the §9 dependency graph). Simplified vs the
// real async DAG: at most one execution per system per tick.
class SystemScheduler {
public:
    void add(System* s, std::vector<std::string> after = {}) {
        entries_.push_back(Entry{s, 1.0 / s->rate_hz(), 0.0, std::move(after)});
        ordered_ = false;
    }
    void tick(World& w, double dt) {
        if (!ordered_) { order(); ordered_ = true; }
        for (Entry* e : run_order_) {
            e->accum += dt;
            if (e->accum + 1e-9 >= e->period) {
                e->sys->update(w, e->accum);
                e->accum = 0.0;
            }
        }
    }

private:
    struct Entry { System* sys; double period; double accum; std::vector<std::string> after; };
    void order() {
        run_order_.clear();
        std::vector<Entry*> remaining;
        for (auto& e : entries_) remaining.push_back(&e);
        std::vector<std::string> done;
        auto ready = [&](Entry* e) {
            for (auto& dep : e->after)
                if (std::find(done.begin(), done.end(), dep) == done.end()) return false;
            return true;
        };
        while (!remaining.empty()) {
            bool progressed = false;
            for (size_t i = 0; i < remaining.size(); ++i) {
                if (ready(remaining[i])) {
                    run_order_.push_back(remaining[i]);
                    done.push_back(remaining[i]->sys->name());
                    remaining.erase(remaining.begin() + i);
                    progressed = true;
                    break;
                }
            }
            if (!progressed) { // unmet dep / cycle: append the rest as-is
                for (auto* e : remaining) run_order_.push_back(e);
                break;
            }
        }
    }
    std::vector<Entry>  entries_;
    std::vector<Entry*> run_order_;
    bool                ordered_ = false;
};

} // namespace mith
