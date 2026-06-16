#pragma once

// Minimal vector / quaternion math for Myth-Eye-Cal.
// Header-only, no dependencies. Used by the keypoint projector and tests.

#include <cmath>

namespace mec {

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
};

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

// Unit quaternion (w, x, y, z), Hamilton convention.
struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat normalized() const {
        const float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n <= 0.0f) return {1.0f, 0.0f, 0.0f, 0.0f};
        const float inv = 1.0f / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }
};

// Rotate vector v by quaternion q (assumes q is, or is close to, unit length).
// v' = v + 2*w*(qv x v) + 2*(qv x (qv x v))
inline Vec3 rotate(const Quat& q, const Vec3& v) {
    const Vec3 qv{q.x, q.y, q.z};
    const Vec3 t = cross(qv, v) * 2.0f;
    return v + t * q.w + cross(qv, t);
}

} // namespace mec
