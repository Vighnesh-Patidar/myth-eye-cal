#pragma once

// LOSDetector — line-of-sight state machine with hysteresis (ARCHITECTURE.md
// §3.2). Header-only, no MithAtomas dependency, so it is unit-testable
// standalone. Driven by ObserverActivationSystem from per-frame pose
// observations; the real observer pipeline feeds the same interface.

#include "mec/types.h"
#include <algorithm>

namespace mec {

struct LOSThresholds {
    float confidence_threshold = 0.6f; // LOS_CONFIDENCE_THRESHOLD
    int   min_keypoints        = 8;    // of 17 above the confidence threshold
    int   acquire_frames       = 10;   // LOS_ACQUIRE_FRAMES
    int   depth_stable_frames  = 5;    // depth stable over >= 5 frames
    float drop_threshold       = 0.3f; // LOS_DROP_THRESHOLD
    int   drop_frames          = 15;   // LOS_DROP_FRAMES
};

class LOSDetector {
public:
    explicit LOSDetector(LOSThresholds t = {}) : t_(t) {}

    LOSState state() const { return state_; }

    // One frame of input. `depth_stable` comes from the depth pipeline.
    LOSState update(const PoseObservation& obs, bool depth_stable) {
        const int n = std::min<int>(obs.keypoint_count, 33);
        int above_conf = 0, above_drop = 0;
        for (int i = 0; i < n; ++i) {
            const float c = obs.keypoints[i].confidence;
            if (c > t_.confidence_threshold) ++above_conf;
            if (c > t_.drop_threshold) ++above_drop;
        }
        const bool strong = above_conf >= t_.min_keypoints;
        const bool weak   = above_drop < t_.min_keypoints;

        if (depth_stable) ++depth_stable_frames_;
        else              depth_stable_frames_ = 0;
        const bool depth_ok = depth_stable_frames_ >= t_.depth_stable_frames;

        switch (state_) {
        case LOSState::NO_TARGET:
            if (strong) { state_ = LOSState::ACQUIRING; visible_frames_ = 1; }
            break;
        case LOSState::ACQUIRING:
            if (strong) {
                ++visible_frames_;
                if (visible_frames_ >= t_.acquire_frames && depth_ok)
                    state_ = LOSState::TRACKING;
            } else {
                state_ = LOSState::NO_TARGET;
                visible_frames_ = 0;
            }
            break;
        case LOSState::TRACKING:
            if (weak) {
                if (++low_frames_ >= t_.drop_frames) {
                    state_ = LOSState::OCCLUDED;
                    low_frames_ = 0;
                }
            } else {
                low_frames_ = 0;
            }
            break;
        case LOSState::OCCLUDED:
            if (strong) state_ = LOSState::TRACKING;      // reacquired
            else        state_ = LOSState::NO_TARGET;     // gave up
            break;
        }
        return state_;
    }

private:
    LOSThresholds t_;
    LOSState state_ = LOSState::NO_TARGET;
    int visible_frames_ = 0;
    int low_frames_ = 0;
    int depth_stable_frames_ = 0;
};

} // namespace mec
