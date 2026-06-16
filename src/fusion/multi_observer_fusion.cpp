#include "mec/fusion/multi_observer_fusion.h"

#include <cmath>

namespace mec {

bool MultiObserverFusion::fuse(const std::vector<WorldKeypoint>& observations,
                               WorldKeypoint& out,
                               float epsilon) {
    if (observations.empty()) return false;

    double sum_w = 0.0;        // sum of position weights
    double sum_wx = 0.0, sum_wy = 0.0, sum_wz = 0.0;
    double sum_inv_var = 0.0;  // sum of 1/sigma^2 for fused uncertainty
    double sum_w_conf = 0.0;   // weight-weighted confidence accumulator
    double latest_ts = 0.0;
    uint8_t id = observations.front().id;

    for (const WorldKeypoint& o : observations) {
        const double sigma2 = static_cast<double>(o.uncertainty_r) * o.uncertainty_r;
        const double w = static_cast<double>(o.confidence) / (sigma2 + epsilon);
        if (w <= 0.0) continue;

        sum_w  += w;
        sum_wx += w * o.wx;
        sum_wy += w * o.wy;
        sum_wz += w * o.wz;
        sum_w_conf += w * o.confidence;

        // Inverse-variance only counts observations with a real uncertainty.
        if (sigma2 > 0.0) sum_inv_var += 1.0 / sigma2;

        if (o.timestamp_s > latest_ts) latest_ts = o.timestamp_s;
    }

    if (sum_w <= 0.0) return false;

    out.id = id;
    out.wx = static_cast<float>(sum_wx / sum_w);
    out.wy = static_cast<float>(sum_wy / sum_w);
    out.wz = static_cast<float>(sum_wz / sum_w);
    out.uncertainty_r = (sum_inv_var > 0.0)
                            ? static_cast<float>(1.0 / std::sqrt(sum_inv_var))
                            : 0.0f;
    out.confidence = static_cast<float>(sum_w_conf / sum_w);
    out.timestamp_s = latest_ts;
    return true;
}

} // namespace mec
