#include "m1_mathematics/Quaternion.h"

#include <cmath>
#include <sstream>

#include "m1_mathematics/Float3.h"
#include "m1_mathematics/MathUtility.h"


using namespace Mach1;

Quaternion::Quaternion() : m_qw(1.0), m_qx(0.0), m_qy(0.0), m_qz(0.0) {}

Quaternion::Quaternion(float qw, float qx, float qy, float qz) : m_qw(qw), m_qx(qx), m_qy(qy), m_qz(qz) {}

Quaternion Quaternion::FromEulerRadians(Float3 euler_vector) {
    // I shamelessly stole this code: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // This is in YXZ format

    float a = euler_vector[1] * 0.5f;
    float b = euler_vector[0] * 0.5f;
    float c = euler_vector[2] * 0.5f;

    float cos_a = cos(a);
    float sin_a = sin(a);

    float cos_b = cos(b);
    float sin_b = sin(b);

    float cos_c = cos(c);
    float sin_c = sin(c);

    return {
            sin_a * sin_b * sin_c + cos_a * cos_b * cos_c,
            sin_a * cos_b * sin_c + cos_a * sin_b * cos_c,
            sin_a * cos_b * cos_c - cos_a * sin_b * sin_c,
            -sin_a * sin_b * cos_c + cos_a * cos_b * sin_c,
    };
}

Quaternion Quaternion::FromEulerDegrees(Float3 euler_vector) {
    return Quaternion::FromEulerRadians(euler_vector.EulerRadians());
}

Float3 Quaternion::ToEulerRadians() {
    // I shamelessly stole this rotation matrix: https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
    // This is in YXZ format

    float s = 2.0f / LengthSquared();

    float xs = m_qx * s;
    float ys = m_qy * s;
    float zs = m_qz * s;

    float wx = m_qw * xs;
    float wy = m_qw * ys;
    float wz = m_qw * zs;

    float xx = m_qx * xs;
    float xy = m_qx * ys;
    float xz = m_qx * zs;

    float yy = m_qy * ys;
    float yz = m_qy * zs;
    float zz = m_qz * zs;

    float p_xx = 1.0f - (yy + zz);
    float p_xy = xy - wz;
    float p_xz = xz + wy;

    float p_yx = xy + wz;
    float p_yy = 1.0f - (xx + zz);
    float p_yz = yz - wx;

    float p_zx = xz - wy;
    float p_zy = yz + wx;
    float p_zz = 1.0f - (xx + yy);

    if (p_yz >= MathUtility::FLOAT_COMPARISON_ONE_MINUS_EPSILON) {
        return {-M_PI_2, -atan2(p_xy, p_xx), 0};
    }

    if (p_yz <= -MathUtility::FLOAT_COMPARISON_ONE_MINUS_EPSILON) {
        return {M_PI_2, atan2(p_xy, p_xx), 0};
    }

    if (p_yx == 0 && p_xy == 0 && p_xz == 0 && p_zx == 0 && p_xx == 1) {
        return {atan2(-p_yz, p_yy), 0, 0};
    }

    return {asin(-p_yz), atan2(p_xz, p_zz), atan2(p_yx, p_yy)};
}

Float3 Quaternion::ToEulerDegrees() {
    return ToEulerRadians().EulerDegrees();
}

bool Quaternion::IsApproximatelyEqual(const Quaternion &rhs) const {
    return MathUtility::IsApproximatelyEqual(m_qw, rhs.m_qw) &&
           MathUtility::IsApproximatelyEqual(m_qx, rhs.m_qx) &&
           MathUtility::IsApproximatelyEqual(m_qy, rhs.m_qy) &&
           MathUtility::IsApproximatelyEqual(m_qz, rhs.m_qz);
}

Quaternion Quaternion::Normalized() const {
    return *this / Length();
}

Quaternion Quaternion::Inversed() const {
    return {m_qw, -m_qx, -m_qy, -m_qz};
}

float Quaternion::DotProduct(Quaternion rhs) const {
    return m_qw * rhs.m_qw + m_qx * rhs.m_qx + m_qy * rhs.m_qy + m_qz * rhs.m_qz;
}

float Quaternion::LengthSquared() const {
    return DotProduct(*this);
}

float Quaternion::Length() const {
    return sqrt(LengthSquared());
}

std::string Quaternion::ToString() const {
    std::stringstream s;
    s << "Quaternion(w: " << m_qw << ", x: " << m_qx << ", y: " << m_qy << ", z: " << m_qz << ")";
    return s.str();
}

// =====================================================================================================================
// ===================================================== OPERATORS =====================================================
// =====================================================================================================================

const float &Quaternion::operator[](int axis) const {
    switch(axis) {
        case 0:
            return m_qw;
        case 1:
            return m_qx;
        case 2:
            return m_qy;
        case 3:
            return m_qz;
        default:
            return m_qw; // for lack of a resolution, other than crashing
    }
}

float &Quaternion::operator[](int axis) {
    switch(axis) {
        case 0:
            return m_qw;
        case 1:
            return m_qx;
        case 2:
            return m_qy;
        case 3:
            return m_qz;
        default:
            return m_qw; // for lack of a resolution, other than crashing
    }
}

bool Quaternion::operator==(const Quaternion &rhs) const {
    return m_qw == rhs.m_qw && m_qx == rhs.m_qx && m_qy == rhs.m_qy && m_qz == rhs.m_qz;
}

bool Quaternion::operator!=(const Quaternion &rhs) const {
    return m_qw != rhs.m_qw || m_qx != rhs.m_qx || m_qy != rhs.m_qy || m_qz != rhs.m_qz;
}

Quaternion Quaternion::operator+(const Quaternion &rhs) const {
    return {m_qw + rhs.m_qw, m_qx + rhs.m_qx, m_qy + rhs.m_qy, m_qz + rhs.m_qz};
}

Quaternion Quaternion::operator-(const Quaternion &rhs) const {
    return {m_qw - rhs.m_qw, m_qx - rhs.m_qx, m_qy - rhs.m_qy, m_qz - rhs.m_qz};
}

void Quaternion::operator*=(const Quaternion &rhs) {
    float a = m_qw * rhs.m_qx + m_qx * rhs.m_qw + m_qy * rhs.m_qz - m_qz * rhs.m_qy;
    float b = m_qw * rhs.m_qy + m_qy * rhs.m_qw + m_qz * rhs.m_qx - m_qx * rhs.m_qz;
    float c = m_qw * rhs.m_qz + m_qz * rhs.m_qw + m_qx * rhs.m_qy - m_qy * rhs.m_qx;
    m_qw = m_qw * rhs.m_qw - m_qx * rhs.m_qx - m_qy * rhs.m_qy - m_qz * rhs.m_qz;
    m_qx = a;
    m_qy = b;
    m_qz = c;
}

Quaternion Quaternion::operator*(const Quaternion &rhs) const {
    Quaternion temp = *this;
    temp *= rhs;
    return temp;
}

void Quaternion::operator*=(float scalar) {
    m_qw *= scalar;
    m_qx *= scalar;
    m_qy *= scalar;
    m_qz *= scalar;
}

void Quaternion::operator/=(float scalar) {
    m_qw /= scalar;
    m_qx /= scalar;
    m_qy /= scalar;
    m_qz /= scalar;
}

Quaternion Quaternion::operator*(float scalar) const {
    return {m_qx * scalar, m_qy * scalar, m_qz * scalar, m_qw * scalar};
}

Quaternion Quaternion::operator/(float scalar) const {
    return {m_qx / scalar, m_qy / scalar, m_qz / scalar, m_qw / scalar};
}
