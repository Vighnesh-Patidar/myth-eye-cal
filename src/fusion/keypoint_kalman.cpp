#include "mec/fusion/keypoint_kalman.h"

#include <algorithm>
#include <cmath>

namespace mec {
namespace {

// --- Minimal row-major 3x3 / 3-vector helpers (zero external deps) ---------
// Destinations must not alias inputs unless noted.

void m_identity(float* M, float s = 1.0f) {
    for (int i = 0; i < 9; ++i) M[i] = 0.0f;
    M[0] = M[4] = M[8] = s;
}
void m_copy(float* d, const float* a) { for (int i = 0; i < 9; ++i) d[i] = a[i]; }
void m_add(float* d, const float* a, const float* b) { for (int i = 0; i < 9; ++i) d[i] = a[i] + b[i]; }
void m_sub(float* d, const float* a, const float* b) { for (int i = 0; i < 9; ++i) d[i] = a[i] - b[i]; }
void m_scale(float* d, const float* a, float s) { for (int i = 0; i < 9; ++i) d[i] = a[i] * s; }

void m_transpose(float* d, const float* a) {
    d[0] = a[0]; d[1] = a[3]; d[2] = a[6];
    d[3] = a[1]; d[4] = a[4]; d[5] = a[7];
    d[6] = a[2]; d[7] = a[5]; d[8] = a[8];
}

// d = a * b (d must not alias a or b).
void m_mul(float* d, const float* a, const float* b) {
    d[0] = a[0]*b[0] + a[1]*b[3] + a[2]*b[6];
    d[1] = a[0]*b[1] + a[1]*b[4] + a[2]*b[7];
    d[2] = a[0]*b[2] + a[1]*b[5] + a[2]*b[8];
    d[3] = a[3]*b[0] + a[4]*b[3] + a[5]*b[6];
    d[4] = a[3]*b[1] + a[4]*b[4] + a[5]*b[7];
    d[5] = a[3]*b[2] + a[4]*b[5] + a[5]*b[8];
    d[6] = a[6]*b[0] + a[7]*b[3] + a[8]*b[6];
    d[7] = a[6]*b[1] + a[7]*b[4] + a[8]*b[7];
    d[8] = a[6]*b[2] + a[7]*b[5] + a[8]*b[8];
}

// d = a * x (3-vector; d must not alias x).
void m_vec(float* d, const float* a, const float* x) {
    d[0] = a[0]*x[0] + a[1]*x[1] + a[2]*x[2];
    d[1] = a[3]*x[0] + a[4]*x[1] + a[5]*x[2];
    d[2] = a[6]*x[0] + a[7]*x[1] + a[8]*x[2];
}

// d = 0.5 (a + aᵀ) — symmetrise to fight round-off (d must not alias a).
void m_symmetrise(float* d, const float* a) {
    d[0] = a[0]; d[4] = a[4]; d[8] = a[8];
    d[1] = d[3] = 0.5f * (a[1] + a[3]);
    d[2] = d[6] = 0.5f * (a[2] + a[6]);
    d[5] = d[7] = 0.5f * (a[5] + a[7]);
}

// General 3x3 inverse via cofactors; returns false if (near-)singular.
bool m_inverse(float* d, const float* a) {
    const float c00 = a[4]*a[8] - a[5]*a[7];
    const float c01 = a[5]*a[6] - a[3]*a[8];
    const float c02 = a[3]*a[7] - a[4]*a[6];
    const float det = a[0]*c00 + a[1]*c01 + a[2]*c02;
    if (std::fabs(det) < 1e-20f) return false;
    const float id = 1.0f / det;
    d[0] = c00 * id;
    d[1] = (a[2]*a[7] - a[1]*a[8]) * id;
    d[2] = (a[1]*a[5] - a[2]*a[4]) * id;
    d[3] = c01 * id;
    d[4] = (a[0]*a[8] - a[2]*a[6]) * id;
    d[5] = (a[2]*a[3] - a[0]*a[5]) * id;
    d[6] = c02 * id;
    d[7] = (a[1]*a[6] - a[0]*a[7]) * id;
    d[8] = (a[0]*a[4] - a[1]*a[3]) * id;
    return true;
}

// Measurement covariance R from the fused observation: the full 3x3 if present
// (cov_xx > 0), otherwise isotropic uncertainty_r^2 * I.
void measurement_cov(const WorldKeypoint& o, float* R) {
    if (o.cov_xx > 0.0f) {
        R[0] = std::max(o.cov_xx, 1e-6f); R[1] = o.cov_xy;               R[2] = o.cov_xz;
        R[3] = o.cov_xy;                  R[4] = std::max(o.cov_yy, 1e-6f); R[5] = o.cov_yz;
        R[6] = o.cov_xz;                  R[7] = o.cov_yz;               R[8] = std::max(o.cov_zz, 1e-6f);
    } else {
        const float r = std::max(o.uncertainty_r * o.uncertainty_r, 1e-6f);
        m_identity(R, r);
    }
}

// A' = A + dt (B + Bᵀ) + dt^2 D + Q_pos, with Q_pos = q dt^4/4 I. Shared by the
// predict step and predict() (which must not mutate state).
void predict_position_cov(const float* A, const float* B, const float* D,
                          float dt, float q, float* A_out) {
    float Bt[9]; m_transpose(Bt, B);
    float tmp[9];
    m_add(tmp, B, Bt);          // B + Bᵀ
    m_scale(tmp, tmp, dt);      // dt (B + Bᵀ)
    m_add(A_out, A, tmp);       // A + dt (B + Bᵀ)
    m_scale(tmp, D, dt * dt);   // dt^2 D
    m_add(A_out, A_out, tmp);
    const float dt2 = dt * dt, dt4 = dt2 * dt2;
    const float qa = q * dt4 * 0.25f;
    A_out[0] += qa; A_out[4] += qa; A_out[8] += qa;
}

} // namespace

KeypointKalmanTracker::KeypointKalmanTracker(float process_accel_std)
    : process_accel_var_(process_accel_std * process_accel_std) {}

void KeypointKalmanTracker::update(const WorldKeypoint& obs) {
    id_ = obs.id;
    last_confidence_ = obs.confidence;

    float R[9];
    measurement_cov(obs, R);

    if (!initialised_) {
        p_[0] = obs.wx; p_[1] = obs.wy; p_[2] = obs.wz;
        v_[0] = v_[1] = v_[2] = 0.0f;
        m_copy(A_, R);               // position cov seeded with measurement cov
        for (int i = 0; i < 9; ++i) B_[i] = 0.0f;
        m_identity(D_, 100.0f);      // large initial velocity variance
        last_update_s_ = obs.timestamp_s;
        initialised_ = true;
        return;
    }

    float dt = static_cast<float>(obs.timestamp_s - last_update_s_);
    if (dt < 0.0f) dt = 0.0f;        // out-of-order frame: don't predict forward

    // --- Predict (state + covariance), F = [[I, dt I],[0, I]] ---
    for (int i = 0; i < 3; ++i) p_[i] += v_[i] * dt;

    float A2[9];
    predict_position_cov(A_, B_, D_, dt, process_accel_var_, A2);

    float B2[9], tmp[9];
    m_scale(tmp, D_, dt);            // dt D
    m_add(B2, B_, tmp);             // B' = B + dt D
    const float dt2 = dt * dt, dt3 = dt2 * dt;
    const float qb = process_accel_var_ * dt3 * 0.5f;
    B2[0] += qb; B2[4] += qb; B2[8] += qb;

    float D2[9]; m_copy(D2, D_);     // D' = D + Q_vel
    const float qd = process_accel_var_ * dt2;
    D2[0] += qd; D2[4] += qd; D2[8] += qd;

    // --- Update with z = position, H = [I 0], measurement cov R ---
    float S[9]; m_add(S, A2, R);     // innovation cov
    float Sinv[9];
    if (!m_inverse(Sinv, S)) {       // singular: keep predicted covariance
        m_copy(A_, A2); m_copy(B_, B2); m_copy(D_, D2);
        last_update_s_ = obs.timestamp_s;
        return;
    }

    float Kp[9]; m_mul(Kp, A2, Sinv);            // Kp = A' S^-1
    float B2t[9]; m_transpose(B2t, B2);
    float Kv[9]; m_mul(Kv, B2t, Sinv);           // Kv = B'ᵀ S^-1

    const float y[3] = { obs.wx - p_[0], obs.wy - p_[1], obs.wz - p_[2] };
    float dp[3], dv[3];
    m_vec(dp, Kp, y); m_vec(dv, Kv, y);
    for (int i = 0; i < 3; ++i) { p_[i] += dp[i]; v_[i] += dv[i]; }

    // Posterior: A+ = (I-Kp)A', B+ = (I-Kp)B', D+ = D' - Kv B'.
    float ImKp[9]; m_identity(ImKp);
    for (int i = 0; i < 9; ++i) ImKp[i] -= Kp[i];

    float Anew[9]; m_mul(Anew, ImKp, A2);
    float Bnew[9]; m_mul(Bnew, ImKp, B2);
    float KvB[9];  m_mul(KvB, Kv, B2);
    float Dnew[9]; m_sub(Dnew, D2, KvB);

    m_symmetrise(A_, Anew);          // A, D stay symmetric; B is a cross block
    m_copy(B_, Bnew);
    m_symmetrise(D_, Dnew);

    last_update_s_ = obs.timestamp_s;
}

WorldKeypoint KeypointKalmanTracker::predict(double timestamp_s) const {
    WorldKeypoint out;
    out.id = id_;
    out.timestamp_s = timestamp_s;

    float dt = static_cast<float>(timestamp_s - last_update_s_);
    if (dt < 0.0f) dt = 0.0f;

    out.wx = p_[0] + v_[0] * dt;
    out.wy = p_[1] + v_[1] * dt;
    out.wz = p_[2] + v_[2] * dt;

    float A2[9];
    predict_position_cov(A_, B_, D_, dt, process_accel_var_, A2);
    out.cov_xx = A2[0]; out.cov_xy = 0.5f * (A2[1] + A2[3]); out.cov_xz = 0.5f * (A2[2] + A2[6]);
    out.cov_yy = A2[4]; out.cov_yz = 0.5f * (A2[5] + A2[7]); out.cov_zz = A2[8];
    out.uncertainty_r = std::sqrt(std::max((A2[0] + A2[4] + A2[8]) / 3.0f, 0.0f));

    const double decay = std::pow(0.5, static_cast<double>(dt) / kConfidenceHalfLifeS);
    out.confidence = static_cast<float>(last_confidence_ * decay);
    return out;
}

} // namespace mec
