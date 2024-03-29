#ifndef M1_ORIENTATIONMANAGER_ORIENTATION_H
#define M1_ORIENTATIONMANAGER_ORIENTATION_H

#include "Float3.h"
#include "Quaternion.h"

namespace Mach1::Math1 {

class Orientation {
public:
    Orientation();

    Float3 GetGlobalRotationAsEulerDegrees() const;

    Float3 GetGlobalRotationAsEulerRadians() const;

    Quaternion GetGlobalRotationAsQuaternion() const;

    void ApplyRotationDegrees(Float3 rotationDegrees);

    void ApplyRotation(Float3 rotationRadians);

    void ApplyRotation(Quaternion quaternion);

    void SetRotation(Float3 rotationRadians);

    void SetRotation(Quaternion quaternion);

    void SetGlobalRotation(Float3 rotationRadians);

    void SetGlobalRotation(Quaternion quaternion);

    void Reset();

    void Recenter();

private:
    Quaternion m_local;
    Quaternion m_parent;
};

} // namespace Mach1::Math1

#endif //M1_ORIENTATIONMANAGER_ORIENTATION_H
