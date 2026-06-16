#pragma once

// MultiObserverFusion - weighted least squares fusion of one keypoint observed
// by N line-of-sight nodes. ARCHITECTURE.md §5.2.

#include "mec/types.h"
#include <vector>

namespace mec {

class MultiObserverFusion {
public:
    // Fuse N observations of a single keypoint into one estimate.
    //
    //   weight_i      = confidence_i / (sigma_i^2 + epsilon)
    //   pos_fused     = sum(w_i * pos_i) / sum(w_i)
    //   sigma_fused   = 1 / sqrt(sum(1 / sigma_i^2))   (inverse-variance)
    //
    // The fused confidence is the weight-weighted mean of input confidences.
    // `out.is_valid` style: returns false and leaves `out` untouched if there
    // are no usable observations (empty input, or all-zero weights).
    //
    // LIMITATION (§ Design Review): sigma_i is an isotropic scalar, so this is
    // a scalar inverse-variance weighted mean. It does not exploit orthogonal
    // viewing geometry per-axis; that requires a 3x3 covariance per observation.
    static bool fuse(const std::vector<WorldKeypoint>& observations,
                     WorldKeypoint& out,
                     float epsilon = 1e-6f);

    // Anisotropic information-form fusion (§15.3). Each observation contributes
    // an information matrix Λ_i; the fused estimate is x = (Σ Λ_i)^-1 (Σ Λ_i x_i)
    // with fused covariance (Σ Λ_i)^-1. An observation with a non-zero view ray
    // (rx,ry,rz) is modelled with variance uncertainty_r^2 perpendicular to the
    // ray and depth_uncertainty^2 along it; one with a zero ray is isotropic
    // (variance uncertainty_r^2). Λ_i is scaled by confidence.
    //
    // This realises §5.2's "geometry bonus": observers at orthogonal angles each
    // constrain the axes they see clearly, so the fusion is well-conditioned in
    // all three axes even though each single view is depth-ambiguous.
    //
    // Returns false if there are no usable observations or Σ Λ_i is singular.
    // The fused `out` carries an isotropic summary uncertainty_r and a zero ray.
    static bool fuse_anisotropic(const std::vector<WorldKeypoint>& observations,
                                 WorldKeypoint& out,
                                 float epsilon = 1e-9f);
};

} // namespace mec
