// Unit tests for the first-party fusion ECS (ecs/world.h): the entity/component
// store, the UserBeaconChannel hook, and the mixed-rate SystemScheduler
// (execution rates, dependency ordering, one-run-per-tick cap).

#include "mec/ecs/world.h"
#include "test_util.h"

#include <string>
#include <vector>

using namespace mec;

namespace {
struct RecSystem : System {
    std::string nm;
    double hz;
    int count = 0;
    std::vector<std::string>* log;
    RecSystem(std::string n, double r, std::vector<std::string>* l = nullptr)
        : nm(std::move(n)), hz(r), log(l) {}
    const char* name() const override { return nm.c_str(); }
    double rate_hz() const override { return hz; }
    void update(World&, double) override { ++count; if (log) log->push_back(nm); }
};
} // namespace

static void test_entity_store() {
    World w;
    const EntityId e1 = w.create_entity();
    const EntityId e2 = w.create_entity();
    CHECK(e1 == 1 && e2 == 2);
    CHECK(e1 != kInvalidEntity);

    CHECK(!w.has<PositionComponent>(e1));
    CHECK(w.get<PositionComponent>(e1) == nullptr); // missing type/entity

    auto& p = w.add<PositionComponent>(e1, PositionComponent{1, 2, 3});
    CHECK(w.has<PositionComponent>(e1));
    CHECK_NEAR(p.x, 1, 1e-6);
    p.x = 9.0f;
    CHECK_NEAR(w.get<PositionComponent>(e1)->x, 9.0, 1e-6); // mutation persists

    // Re-add overwrites the stored value.
    w.add<PositionComponent>(e1, PositionComponent{4, 5, 6});
    CHECK_NEAR(w.get<PositionComponent>(e1)->x, 4.0, 1e-6);

    // get_or_add default-constructs on demand.
    auto& b = w.get_or_add<BehaviourStateComponent>(e2);
    CHECK(b.los_state == 0);
    b.los_state = 2;
    CHECK(w.get_or_add<BehaviourStateComponent>(e2).los_state == 2);

    // Components are keyed per-entity.
    CHECK(!w.has<PositionComponent>(e2));

    w.set_local(e1);
    CHECK(w.local() == e1);
    w.set_now(12.5);
    CHECK_NEAR(w.now_s(), 12.5, 1e-9);
}

static void test_user_beacon_channel() {
    World w;
    std::array<uint8_t, 128> captured{};
    bool called = false;
    w.user_beacon.on_broadcast = [&](const std::array<uint8_t, 128>& p) {
        captured = p; called = true;
    };
    std::array<uint8_t, 128> payload{};
    payload[0] = 0xAB; payload[127] = 0xCD;

    CHECK(!w.user_beacon.sent);
    w.user_beacon.broadcast(payload);
    CHECK(w.user_beacon.sent);
    CHECK(w.user_beacon.count == 1);
    CHECK(called);
    CHECK(captured[0] == 0xAB && captured[127] == 0xCD);
    CHECK(w.user_beacon.last_payload[0] == 0xAB);

    w.user_beacon.broadcast(payload);
    CHECK(w.user_beacon.count == 2);
}

static void test_scheduler_rates() {
    World w;
    RecSystem fast("fast", 60.0), slow("slow", 10.0);
    SystemScheduler sched;
    sched.add(&fast);
    sched.add(&slow);

    const double dt = 1.0 / 60.0;
    for (int i = 0; i < 60; ++i) sched.tick(w, dt); // 1 second
    CHECK(fast.count == 60);             // runs every tick
    CHECK(slow.count >= 9 && slow.count <= 11); // ~10Hz
}

static void test_scheduler_ordering() {
    World w;
    std::vector<std::string> log;
    RecSystem a("A", 1.0, &log), b("B", 1.0, &log), c("C", 1.0, &log);
    SystemScheduler sched;
    // Added out of dependency order on purpose; the scheduler must topo-sort.
    sched.add(&c, {"B"});
    sched.add(&a);
    sched.add(&b, {"A"});

    sched.tick(w, 1.0); // period 1s, dt 1s -> all run exactly once
    CHECK(log.size() == 3);
    CHECK(a.count == 1 && b.count == 1 && c.count == 1);
    // A before B before C.
    CHECK(log[0] == "A");
    CHECK(log[1] == "B");
    CHECK(log[2] == "C");
}

static void test_scheduler_one_run_per_tick() {
    World w;
    RecSystem s("once", 1.0);
    SystemScheduler sched;
    sched.add(&s);
    sched.tick(w, 10.0); // 10s of accumulated time in one tick
    CHECK(s.count == 1); // capped at one execution per tick
}

int main() {
    test_entity_store();
    test_user_beacon_channel();
    test_scheduler_rates();
    test_scheduler_ordering();
    test_scheduler_one_run_per_tick();
    RUN_TESTS_RETURN();
}
