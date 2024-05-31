#include "m1_mathematics/Orientation.h"

using namespace Mach1;

Orientation::Orientation() : m_local(), m_parent() {
}

Quaternion Orientation::GetGlobalRotationAsQuaternion() const {
    return m_parent * m_local;
}

Float3 Orientation:: GetGlobalRotationAsEulerDegrees() const {
    return GetGlobalRotationAsQuaternion().ToEulerDegrees();
}

Float3 Orientation::GetGlobalRotationAsEulerRadians() const {
    return GetGlobalRotationAsQuaternion().ToEulerRadians();
}

void Orientation::ApplyRotation(Quaternion quaternion) {
    m_local *= quaternion;
}

void Orientation::ApplyRotationDegrees(Float3 rotationDegrees) {
    return ApplyRotation(Quaternion::FromEulerDegrees(rotationDegrees));
}
void Orientation::ApplyRotationDegrees_YawAxis(float yaw) { return ApplyRotation(Quaternion::FromEulerDegrees({yaw, 0, 0})); }
void Orientation::ApplyRotationDegrees_PitchAxis(float pitch) { return ApplyRotation(Quaternion::FromEulerDegrees({0, pitch, 0})); }
void Orientation::ApplyRotationDegrees_RollAxis(float roll) { return ApplyRotation(Quaternion::FromEulerDegrees({0, 0, roll})); }

void Orientation::ApplyRotation(Float3 rotationRadians) {
    return ApplyRotation(Quaternion::FromEulerRadians(rotationRadians));
}
void Orientation::ApplyRotation_YawAxis(float yaw) { return ApplyRotation(Quaternion::FromEulerRadians({yaw, 0, 0})); }
void Orientation::ApplyRotation_PitchAxis(float pitch) { return ApplyRotation(Quaternion::FromEulerRadians({0, pitch, 0})); }
void Orientation::ApplyRotation_RollAxis(float roll) { return ApplyRotation(Quaternion::FromEulerRadians({0, 0, roll})); }

void Orientation::Recenter() {
    m_parent = m_local.Inversed();
}

void Orientation::Reset() {
    m_local = {};
    m_parent = {};
}

void Orientation::SetRotation(Quaternion quaternion) {
    m_local = quaternion;
}

void Orientation::SetRotation(Float3 rotationRadians) {
    SetRotation(Quaternion::FromEulerRadians(rotationRadians));
}

void Orientation::SetGlobalRotation(Float3 rotationRadians) {
    Reset();
    SetRotation(rotationRadians);
}

void Orientation::SetGlobalRotation(Quaternion quaternion) {
    Reset();
    SetRotation(quaternion);
}
