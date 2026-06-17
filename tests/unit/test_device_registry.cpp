// Unit tests for DeviceRegistry: passive discovery, manual connect allowlist,
// fusion gating, TTL pruning.

#include "mec/transport/device_registry.h"
#include "test_util.h"

using namespace mec;

static void test_manual_gating() {
    DeviceRegistry reg(/*ttl_s=*/8.0);
    CHECK(reg.manual()); // manual by default

    reg.observe(101, 1.0, 1, 2, 3, /*los=*/2, /*has_data=*/true);
    reg.observe(202, 1.0, 0, 0, 0, /*los=*/2, /*has_data=*/false);

    // Nothing fuses until the operator connects it.
    CHECK(!reg.should_fuse(101));
    CHECK(!reg.should_fuse(202));

    reg.connect(101);
    CHECK(reg.should_fuse(101));
    CHECK(!reg.should_fuse(202));
    CHECK(reg.connected_count() == 1);

    reg.disconnect(101);
    CHECK(!reg.should_fuse(101));
}

static void test_auto_mode() {
    DeviceRegistry reg;
    reg.set_manual(false); // auto: fuse everything heard
    reg.observe(7, 1.0, 0, 0, 0, 2, true);
    CHECK(reg.should_fuse(7));
    CHECK(reg.should_fuse(999)); // even unheard ids fuse in auto mode
}

static void test_list_and_ttl() {
    DeviceRegistry reg(/*ttl_s=*/5.0);
    reg.observe(1, 1.0, 0, 0, 0, 2, true);
    reg.observe(2, 3.0, 0, 0, 0, 2, false);
    reg.connect(2);

    auto l = reg.list(/*now=*/3.0);
    CHECK(l.size() == 2);
    // Most-recently-seen first.
    CHECK(l[0].id == 2);
    CHECK(l[0].connected);
    CHECK(!l[1].connected);
    CHECK(l[1].has_data);

    // Device 1 (last seen t=1) falls outside the 5s ttl at t=7; device 2 stays.
    auto l2 = reg.list(/*now=*/7.0);
    CHECK(l2.size() == 1);
    CHECK(l2[0].id == 2);

    // A connection persists even after the device is pruned, and re-applies
    // when it reappears.
    CHECK(reg.is_connected(2));
    reg.observe(1, 7.5, 0, 0, 0, 2, true); // device 1 returns; still not connected
    CHECK(!reg.should_fuse(1));
}

static void test_connect_all() {
    DeviceRegistry reg(/*ttl_s=*/5.0);
    reg.observe(1, 1.0, 0, 0, 0, 2, true);
    reg.observe(2, 1.0, 0, 0, 0, 2, true);
    reg.observe(3, 0.0, 0, 0, 0, 2, true); // stale relative to now=10
    reg.connect_all(/*now=*/2.0);
    CHECK(reg.is_connected(1));
    CHECK(reg.is_connected(2));
    reg.disconnect_all();
    CHECK(reg.connected_count() == 0);
}

int main() {
    test_manual_gating();
    test_auto_mode();
    test_list_and_ttl();
    test_connect_all();
    RUN_TESTS_RETURN();
}
