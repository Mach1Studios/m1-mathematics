#include "m1_mathematics/Orientation.h"

using namespace Mach1;
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

void Orientation::ApplyRotation(Float3 rotationRadians) {
    return ApplyRotation(Quaternion::FromEulerRadians(rotationRadians));
}

void Orientation::Recenter() {
    m_parent = GetGlobalRotationAsQuaternion().Inversed();
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


