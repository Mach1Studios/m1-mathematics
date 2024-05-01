#include "m1_mathematics/Float3.h"
#include "m1_mathematics/MathUtility.h"
#include <sstream>
#include <cmath>

using namespace Mach1;


Float3::Float3() : m_x(0), m_y(0), m_z(0) {}

Float3::Float3(float x, float y, float z) : m_x(x), m_y(y), m_z(z) {}

Float3::Float3(float component) : m_x(component), m_y(component), m_z(component) {}

float Float3::Length() const {
    return sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
}

Float3 Float3::Normalized() const {
    float length_squared = m_x * m_x + m_y * m_y + m_z * m_z;

    if (length_squared == 0) {
        return {};
    }

    float length = sqrt(length_squared);
    return *this / length;
}


Float3 Float3::Map(float from_min, float from_max, float to_min, float to_max) {
    float from_range = from_max - from_min;

    if (from_range == 0) {
        return {};
    }

    float to_range = to_max - to_min;
    return ((*this - from_min) / from_range * to_range) + to_min;
}


Float3 Float3::Clamped(Float3 min, Float3 max) const {
    return {
            std::clamp(m_x, min.m_x, max.m_x),
            std::clamp(m_y, min.m_y, max.m_y),
            std::clamp(m_z, min.m_z, max.m_z)
    };
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
    return MathUtility::IsApproximatelyEqual(m_x, rhs.m_x) &&
           MathUtility::IsApproximatelyEqual(m_y, rhs.m_y) &&
           MathUtility::IsApproximatelyEqual(m_z, rhs.m_z);
}

std::string Float3::ToString() const {
    std::stringstream s;
    s << "Float3(" << m_x << ", " << m_y << ", " << m_z << ")";
    return s.str();
}

float Float3::GetYaw() const {
    return m_y;
}

float Float3::GetPitch() const {
    return m_x;
}

float Float3::GetRoll() const {
    return m_z;
}

// =====================================================================================================================
// ===================================================== OPERATORS =====================================================
// =====================================================================================================================

Float3 &Float3::operator+=(const Float3 &rhs) {
    m_x += rhs.m_x;
    m_y += rhs.m_y;
    m_z += rhs.m_z;
    return *this;
}

Float3 &Float3::operator-=(const Float3 &rhs) {
    m_x -= rhs.m_x;
    m_y -= rhs.m_y;
    m_z -= rhs.m_z;
    return *this;
}

Float3 &Float3::operator*=(const Float3 &rhs) {
    m_x *= rhs.m_x;
    m_y *= rhs.m_y;
    m_z *= rhs.m_z;
    return *this;
}

Float3 &Float3::operator/=(const Float3 &rhs) {
    m_x /= rhs.m_x;
    m_y /= rhs.m_y;
    m_z /= rhs.m_z;
    return *this;
}

Float3 &Float3::operator*=(float rhs_scalar) {
    m_x *= rhs_scalar;
    m_y *= rhs_scalar;
    m_z *= rhs_scalar;
    return *this;
}

Float3 &Float3::operator/=(float rhs_scalar) {
    m_x /= rhs_scalar;
    m_y /= rhs_scalar;
    m_z /= rhs_scalar;
    return *this;
}

Float3 Float3::operator+(const Float3 &rhs) const {
    return {m_x + rhs.m_x, m_y + rhs.m_y, m_z + rhs.m_z};
}

Float3 Float3::operator-(const Float3 &rhs) const {
    return {m_x - rhs.m_x, m_y - rhs.m_y, m_z - rhs.m_z};
}

Float3 Float3::operator*(const Float3 &rhs) const {
    return {m_x * rhs.m_x, m_y * rhs.m_y, m_z * rhs.m_z};
}

Float3 Float3::operator/(const Float3 &rhs) const {
    return {m_x / rhs.m_x, m_y / rhs.m_y, m_z / rhs.m_z};
}

Float3 Float3::operator*(float rhs_scalar) const {
    return {m_x * rhs_scalar, m_y * rhs_scalar, m_z * rhs_scalar};
}

Float3 Float3::operator/(float rhs_scalar) const {
    return {m_x / rhs_scalar, m_y / rhs_scalar, m_z / rhs_scalar};
}

const float &Float3::operator[](int axis) const {
    switch (axis) {
        case 0:
            return m_x;
        case 1:
            return m_y;
        case 2:
            return m_z;
        default:
            return m_x; // for lack of a resolution, other than crashing
    }
}

float &Float3::operator[](int axis) {
    switch (axis) {
        case 0:
            return m_x;
        case 1:
            return m_y;
        case 2:
            return m_z;
        default:
            return m_x; // for lack of a resolution, other than crashing
    }
}

bool Float3::operator==(const Float3 &rhs) const {
    return (m_x == rhs.m_x) && (m_y == rhs.m_y) && (m_z == rhs.m_z);
}

bool Float3::operator!=(const Float3 &rhs) const {
    return (m_x != rhs.m_x) || (m_y != rhs.m_y) || (m_z != rhs.m_z);
}

Float3 Float3::operator+(float rhs_scalar) const {
    return {m_x + rhs_scalar, m_y + rhs_scalar, m_z + rhs_scalar};
}

Float3 Float3::operator-(float rhs_scalar) const {
    return {m_x - rhs_scalar, m_y - rhs_scalar, m_z - rhs_scalar};
}
