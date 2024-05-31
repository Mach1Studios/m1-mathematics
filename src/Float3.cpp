#include "m1_mathematics/Float3.h"
#include "m1_mathematics/MathUtility.h"
#include <sstream>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif 

using namespace Mach1;

Float3::Float3() : m_yaw(0), m_pitch(0), m_roll(0) {}
Float3::Float3(float yaw, float pitch, float roll) : m_yaw(yaw), m_pitch(pitch), m_roll(roll) {}
Float3::Float3(float component) : m_yaw(component), m_pitch(component), m_roll(component) {}

float Float3::Length() const {
    return sqrt(m_yaw * m_yaw + m_pitch * m_pitch + m_roll * m_roll);
}

Float3 Float3::Normalized() const {
    float length_squared = m_yaw * m_yaw + m_pitch * m_pitch + m_roll * m_roll;

    if (length_squared == 0) {
        return {};
    }

    float length = sqrt(length_squared);
    return *this / length;
}

Float3 Float3::Clamped(Float3 min, Float3 max) const {
    return {
        std::clamp(m_yaw, min.m_yaw, max.m_yaw),
        std::clamp(m_pitch, min.m_pitch, max.m_pitch),
        std::clamp(m_roll, min.m_roll, max.m_roll)
    };
}

Float3 Float3::Modulus(Float3 min_fmod, Float3 max_fmod) const {
    return {
        // Performs modulus with entire range and then offsets the result by the minimum
        (m_yaw > max_fmod.m_yaw) ? std::fmod(m_yaw, max_fmod.m_yaw - min_fmod.m_yaw) + min_fmod.m_yaw : std::fmod(m_yaw, max_fmod.m_yaw - min_fmod.m_yaw),
        (m_pitch > max_fmod.m_pitch) ? std::fmod(m_pitch, max_fmod.m_pitch - min_fmod.m_pitch) + min_fmod.m_pitch : std::fmod(m_pitch, max_fmod.m_pitch - min_fmod.m_pitch),
        (m_roll > max_fmod.m_roll) ? std::fmod(m_roll, max_fmod.m_roll - min_fmod.m_roll) + min_fmod.m_roll : std::fmod(m_roll, max_fmod.m_roll - min_fmod.m_roll)
    };
}

Float3 Float3::Map(float from_min, float from_max, float to_min, float to_max) {
    float from_range = from_max - from_min;

    if (from_range == 0) {
        return {};
    }

    float to_range = to_max - to_min;
    return ((*this - from_min) / from_range * to_range) + to_min;
}

Float3 Float3::EulerDegrees() const {
    static const float toDegConst = 180.0 / M_PI;
    return *this * toDegConst;
}

Float3 Float3::EulerRadians() const {
    static const float toRadConst = M_PI / 180.0;
    return *this * toRadConst;
}

bool Float3::IsApproximatelyEqual(const Float3 &rhs) const {
    return MathUtility::IsApproximatelyEqual(m_yaw, rhs.m_yaw) &&
           MathUtility::IsApproximatelyEqual(m_pitch, rhs.m_pitch) &&
           MathUtility::IsApproximatelyEqual(m_roll, rhs.m_roll);
}

std::string Float3::ToString() const {
    std::stringstream s;
    s << "Float3(" << m_yaw << ", " << m_pitch << ", " << m_roll << ")";
    return s.str();
}

float Float3::GetYaw() const {
    return m_yaw;
}

float Float3::GetPitch() const {
    return m_pitch;
}

float Float3::GetRoll() const {
    return m_roll;
}

// =====================================================================================================================
// ===================================================== OPERATORS =====================================================
// =====================================================================================================================

Float3 &Float3::operator+=(const Float3 &rhs) {
    m_yaw += rhs.m_yaw;
    m_pitch += rhs.m_pitch;
    m_roll += rhs.m_roll;
    return *this;
}

Float3 &Float3::operator-=(const Float3 &rhs) {
    m_yaw -= rhs.m_yaw;
    m_pitch -= rhs.m_pitch;
    m_roll -= rhs.m_roll;
    return *this;
}

Float3 &Float3::operator*=(const Float3 &rhs) {
    m_yaw *= rhs.m_yaw;
    m_pitch *= rhs.m_pitch;
    m_roll *= rhs.m_roll;
    return *this;
}

Float3 &Float3::operator/=(const Float3 &rhs) {
    m_yaw /= rhs.m_yaw;
    m_pitch /= rhs.m_pitch;
    m_roll /= rhs.m_roll;
    return *this;
}

Float3 &Float3::operator*=(float rhs_scalar) {
    m_yaw *= rhs_scalar;
    m_pitch *= rhs_scalar;
    m_roll *= rhs_scalar;
    return *this;
}

Float3 &Float3::operator/=(float rhs_scalar) {
    m_yaw /= rhs_scalar;
    m_pitch /= rhs_scalar;
    m_roll /= rhs_scalar;
    return *this;
}

Float3 Float3::operator+(const Float3 &rhs) const {
    return {m_yaw + rhs.m_yaw, m_pitch + rhs.m_pitch, m_roll + rhs.m_roll};
}

Float3 Float3::operator-(const Float3 &rhs) const {
    return {m_yaw - rhs.m_yaw, m_pitch - rhs.m_pitch, m_roll - rhs.m_roll};
}

Float3 Float3::operator*(const Float3 &rhs) const {
    return {m_yaw * rhs.m_yaw, m_pitch * rhs.m_pitch, m_roll * rhs.m_roll};
}

Float3 Float3::operator/(const Float3 &rhs) const {
    return {m_yaw / rhs.m_yaw, m_pitch / rhs.m_pitch, m_roll / rhs.m_roll};
}

Float3 Float3::operator*(float rhs_scalar) const {
    return {m_yaw * rhs_scalar, m_pitch * rhs_scalar, m_roll * rhs_scalar};
}

Float3 Float3::operator/(float rhs_scalar) const {
    return {m_yaw / rhs_scalar, m_pitch / rhs_scalar, m_roll / rhs_scalar};
}

const float &Float3::operator[](int axis) const {
    switch (axis) {
        case 0:
            return m_yaw;
        case 1:
            return m_pitch;
        case 2:
            return m_roll;
        default:
            return m_yaw; // for lack of a resolution, other than crashing
    }
}

float &Float3::operator[](int axis) {
    switch (axis) {
        case 0:
            return m_yaw;
        case 1:
            return m_pitch;
        case 2:
            return m_roll;
        default:
            return m_yaw; // for lack of a resolution, other than crashing
    }
}

bool Float3::operator==(const Float3 &rhs) const {
    return (m_yaw == rhs.m_yaw) && (m_pitch == rhs.m_pitch) && (m_roll == rhs.m_roll);
}

bool Float3::operator!=(const Float3 &rhs) const {
    return (m_yaw != rhs.m_yaw) || (m_pitch != rhs.m_pitch) || (m_roll != rhs.m_roll);
}

Float3 Float3::operator+(float rhs_scalar) const {
    return {m_yaw + rhs_scalar, m_pitch + rhs_scalar, m_roll + rhs_scalar};
}

Float3 Float3::operator-(float rhs_scalar) const {
    return {m_yaw - rhs_scalar, m_pitch - rhs_scalar, m_roll - rhs_scalar};
}
