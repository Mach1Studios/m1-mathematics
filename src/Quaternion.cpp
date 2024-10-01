#include "m1_mathematics/Quaternion.h"

#include <cmath>
#include <sstream>

#include "m1_mathematics/Float3.h"
#include "m1_mathematics/MathUtility.h"

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

using namespace Mach1;

Quaternion::Quaternion() : m_qw(1.0), m_qx(0.0), m_qy(0.0), m_qz(0.0) {}

Quaternion::Quaternion(float qw, float qx, float qy, float qz) : m_qw(qw), m_qx(qx), m_qy(qy), m_qz(qz) {}

Quaternion Quaternion::FromEulerRadians(Float3 euler_vector) {
    // Convert to half angles
    float yaw = euler_vector[0] * 0.5f;
    float pitch = euler_vector[1] * 0.5f;
    float roll = euler_vector[2] * 0.5f;

    // Compute cosines and sines of half angles
    float cosYaw = cos(yaw);
    float sinYaw = sin(yaw);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);
    float cosRoll = cos(roll);
    float sinRoll = sin(roll);

    // Quaternion components computed in Yaw-Pitch-Roll order (Z-Y-X)
    float qw = cosYaw * cosPitch * cosRoll + sinYaw * sinPitch * sinRoll;
    float qx = cosYaw * cosPitch * sinRoll - sinYaw * sinPitch * cosRoll;
    float qy = cosYaw * sinPitch * cosRoll + sinYaw * cosPitch * sinRoll;
    float qz = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;

    return { qw, qx, qy, qz };
}

Quaternion Quaternion::FromEulerDegrees(Float3 euler_vector) {
    return Quaternion::FromEulerRadians(euler_vector.EulerRadians());
}

Float3 Quaternion::ToEulerRadians() {
    // Normalize the quaternion
    float norm = sqrt(m_qw * m_qw + m_qx * m_qx + m_qy * m_qy + m_qz * m_qz);
    float qw = m_qw / norm;
    float qx = m_qx / norm;
    float qy = m_qy / norm;
    float qz = m_qz / norm;

    // Precompute repeated values
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);

    float sinp = 2.0f * (qw * qy - qz * qx);
    // Handle gimbal lock
    float pitch;
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(M_PI_2, sinp); // Use 90 degrees if out of range
    else
        pitch = asin(sinp);

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

    // Compute Euler angles
    float yaw = atan2(siny_cosp, cosy_cosp);   // Yaw (ψ)
    float roll = atan2(sinr_cosp, cosr_cosp);  // Roll (φ)

    // Return angles in Yaw-Pitch-Roll order
    return { yaw, pitch, roll };
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

float Quaternion::GetW() const {
    return m_qw;
}

float Quaternion::GetX() const {
    return m_qx;
}

float Quaternion::GetY() const {
    return m_qy;
}

float Quaternion::GetZ() const {
    return m_qz;
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
