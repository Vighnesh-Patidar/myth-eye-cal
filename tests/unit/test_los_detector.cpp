#include "mec/observer/los_detector.h"
#include "test_util.h"

using namespace mec;

static PoseObservation obs_with_conf(float conf, int count = 33) {
    PoseObservation o;
    o.keypoint_count = static_cast<uint8_t>(count);
    for (int i = 0; i < count && i < 33; ++i) o.keypoints[i].confidence = conf;
    return o;
}

int main() {
    const PoseObservation strong = obs_with_conf(0.9f); // > 0.6 threshold
    const PoseObservation weak   = obs_with_conf(0.1f); // < 0.3 drop threshold

    // No target while confidence is low.
    {
        LOSDetector d;
        d.update(weak, false);
        CHECK(d.state() == LOSState::NO_TARGET);
    }

    // Acquire -> Tracking after acquire_frames (10) with stable depth.
    {
        LOSDetector d;
        for (int i = 0; i < 9; ++i) d.update(strong, true);
        CHECK(d.state() == LOSState::ACQUIRING);
        d.update(strong, true);
        CHECK(d.state() == LOSState::TRACKING);
    }

    // Hysteresis: stays TRACKING through brief weakness, drops after drop_frames (15).
    {
        LOSDetector d;
        for (int i = 0; i < 10; ++i) d.update(strong, true);
        CHECK(d.state() == LOSState::TRACKING);
        for (int i = 0; i < 14; ++i) d.update(weak, true);
        CHECK(d.state() == LOSState::TRACKING); // not yet dropped
        d.update(weak, true);
        CHECK(d.state() == LOSState::OCCLUDED);
    }

    // Depth never stabilising blocks promotion to TRACKING.
    {
        LOSDetector d;
        for (int i = 0; i < 30; ++i) d.update(strong, false);
        CHECK(d.state() == LOSState::ACQUIRING);
    }

    RUN_TESTS_RETURN();
}
