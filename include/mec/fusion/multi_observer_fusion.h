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
};

} // namespace mec
