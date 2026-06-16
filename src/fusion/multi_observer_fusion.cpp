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

bool MultiObserverFusion::fuse_anisotropic(const std::vector<WorldKeypoint>& observations,
                                           WorldKeypoint& out, float epsilon) {
    if (observations.empty()) return false;

    // Symmetric information matrix L (a,b,c / b,d,e / c,e,f) and vector eta.
    double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
    double e0 = 0, e1 = 0, e2 = 0;
    double sum_conf = 0, latest_ts = 0;
    int used = 0;
    const uint8_t id = observations.front().id;

    for (const WorldKeypoint& o : observations) {
        if (o.confidence <= 0.0f) continue;
        const double conf = o.confidence;
        const double rn2 = static_cast<double>(o.rx) * o.rx +
                           static_cast<double>(o.ry) * o.ry +
                           static_cast<double>(o.rz) * o.rz;

        double Li00, Li01, Li02, Li11, Li12, Li22;
        if (rn2 > 1e-12) {
            // Anisotropic: Λ_i = w_lat (I - r rᵀ) + w_depth (r rᵀ)
            //                  = w_lat I + (w_depth - w_lat) r rᵀ.
            const double inv = 1.0 / std::sqrt(rn2);
            const double rx = o.rx * inv, ry = o.ry * inv, rz = o.rz * inv;
            const double sl2 = std::max(static_cast<double>(o.uncertainty_r) * o.uncertainty_r, 1e-12);
            const double sd2 = std::max(static_cast<double>(o.depth_uncertainty) * o.depth_uncertainty, 1e-12);
            const double w_lat = conf / sl2;
            const double w_depth = conf / sd2;
            const double k = w_depth - w_lat;
            Li00 = w_lat + k * rx * rx; Li11 = w_lat + k * ry * ry; Li22 = w_lat + k * rz * rz;
            Li01 = k * rx * ry; Li02 = k * rx * rz; Li12 = k * ry * rz;
        } else {
            // Isotropic: Λ_i = (conf / sigma^2) I.
            const double s2 = std::max(static_cast<double>(o.uncertainty_r) * o.uncertainty_r, 1e-12);
            const double w = conf / s2;
            Li00 = Li11 = Li22 = w; Li01 = Li02 = Li12 = 0.0;
        }

        a += Li00; b += Li01; c += Li02; d += Li11; e += Li12; f += Li22;
        // eta += Λ_i * x_i
        const double x = o.wx, y = o.wy, z = o.wz;
        e0 += Li00 * x + Li01 * y + Li02 * z;
        e1 += Li01 * x + Li11 * y + Li12 * z;
        e2 += Li02 * x + Li12 * y + Li22 * z;

        sum_conf += conf;
        if (o.timestamp_s > latest_ts) latest_ts = o.timestamp_s;
        ++used;
    }
    if (used == 0) return false;

    // Invert the symmetric 3x3 information matrix.
    const double adj00 = d * f - e * e;
    const double adj01 = c * e - b * f;
    const double adj02 = b * e - c * d;
    const double adj11 = a * f - c * c;
    const double adj12 = b * c - a * e;
    const double adj22 = a * d - b * b;
    const double det = a * adj00 + b * adj01 + c * adj02;
    if (std::fabs(det) < epsilon) return false;
    const double idet = 1.0 / det;

    const double i00 = adj00 * idet, i01 = adj01 * idet, i02 = adj02 * idet;
    const double i11 = adj11 * idet, i12 = adj12 * idet, i22 = adj22 * idet;

    out.id = id;
    out.wx = static_cast<float>(i00 * e0 + i01 * e1 + i02 * e2);
    out.wy = static_cast<float>(i01 * e0 + i11 * e1 + i12 * e2);
    out.wz = static_cast<float>(i02 * e0 + i12 * e1 + i22 * e2);
    // Isotropic summary of the fused covariance (RMS of the principal axes).
    out.uncertainty_r = static_cast<float>(std::sqrt(std::max((i00 + i11 + i22) / 3.0, 0.0)));
    out.rx = out.ry = out.rz = 0.0f;
    out.depth_uncertainty = 0.0f;
    out.confidence = static_cast<float>(sum_conf / used);
    out.timestamp_s = latest_ts;
    return true;
}

} // namespace mec
