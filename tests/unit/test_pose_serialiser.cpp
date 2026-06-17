// Tests for the §6.3 "pose_frame" JSON serialiser (render/pose_serialiser.h).
// Verifies the envelope, the keypoint array length/fields, and numeric
// formatting, without pulling in a JSON parser (string-contains checks).

#include "mec/render/pose_serialiser.h"
#include "test_util.h"

#include <string>

using namespace mec;

static size_t count_occurrences(const std::string& hay, const std::string& needle) {
    size_t n = 0, pos = 0;
    while ((pos = hay.find(needle, pos)) != std::string::npos) { ++n; pos += needle.size(); }
    return n;
}

int main() {
    FusedPose pose;
    pose.timestamp_s = 1.5;
    pose.observer_count = 3;
    pose.mean_confidence = 0.5f;
    pose.is_valid = true;
    for (int i = 0; i < kNumKeypoints; ++i) {
        pose.keypoints[i].id = static_cast<uint8_t>(i);
        pose.keypoints[i].wx = 0.1f * i;
        pose.keypoints[i].wy = -0.2f * i;
        pose.keypoints[i].wz = 1.0f + i;
        pose.keypoints[i].confidence = 0.8f;
    }

    const std::string js = serialise_pose(pose);

    // Envelope.
    CHECK(js.find("\"type\":\"pose_frame\"") != std::string::npos);
    CHECK(js.find("\"timestamp_s\":1.500") != std::string::npos); // %.3f
    CHECK(js.find("\"observer_count\":3") != std::string::npos);
    CHECK(js.find("\"keypoints\":[") != std::string::npos);

    // Exactly kNumKeypoints entries, each with the full field set.
    CHECK(count_occurrences(js, "\"id\":") == kNumKeypoints);
    CHECK(count_occurrences(js, "\"wx\":") == kNumKeypoints);
    CHECK(count_occurrences(js, "\"wy\":") == kNumKeypoints);
    CHECK(count_occurrences(js, "\"wz\":") == kNumKeypoints);
    CHECK(count_occurrences(js, "\"conf\":") == kNumKeypoints);

    // A representative value renders correctly (kp 10: wz = 11.0, conf 0.8).
    CHECK(js.find("\"wz\":11.000") != std::string::npos);
    CHECK(js.find("\"conf\":0.800") != std::string::npos);

    // Well-formed: balanced single top-level object and array.
    CHECK(js.front() == '{' && js.back() == '}');
    CHECK(count_occurrences(js, "[") == 1 && count_occurrences(js, "]") == 1);

    RUN_TESTS_RETURN();
}
