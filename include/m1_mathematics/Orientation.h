#ifndef M1_ORIENTATIONMANAGER_ORIENTATION_H
#define M1_ORIENTATIONMANAGER_ORIENTATION_H

#include "Float3.h"
#include "Quaternion.h"

namespace Mach1 {

class Orientation {
public:
    Orientation();

    /**
     * @brief Get the absolute rotation of this Orientation as a Euler degrees Float3, which
     * equates to this Orientation's local Quaternion, relative to the parent Quaternion
     */
    Float3 GetGlobalRotationAsEulerDegrees() const;

    /**
     * @brief Get the absolute rotation of this Orientation as a Euler radians Float3, which
     * equates to this Orientation's local Quaternion, relative to the parent Quaternion
     */
    Float3 GetGlobalRotationAsEulerRadians() const;

    /**
     * @brief Get the absolute rotation of this Orientation as a Quaternion, which
     * equates to this Orientation's local Quaternion, relative to the parent Quaternion
     */
    Quaternion GetGlobalRotationAsQuaternion() const;

    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Quaternion adding it to the current rotation
     */
    void ApplyRotation(Quaternion quaternion);

    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler degrees Float3 adding it to the current rotation
     */
    void ApplyRotationDegrees(Float3 rotationDegrees);

    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler degrees around the Yaw axis only
     * adding it to the current rotation
     */
    void ApplyRotationDegrees_YawAxis(float yaw);
    
    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler degrees around the Pitch axis only
     * adding it to the current rotation
     */
    void ApplyRotationDegrees_PitchAxis(float pitch);
    
    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler degrees around the Roll axis only
     * adding it to the current rotation
     */
    void ApplyRotationDegrees_RollAxis(float roll);

    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler radians Float3 adding it to the current rotation
     */
    void ApplyRotation(Float3 rotationRadians);

    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler radians around the Yaw axis only
     * adding it to the current rotation
     */
    void ApplyRotation_YawAxis(float yaw);
    
    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler radians around the Pitch axis only
     * adding it to the current rotation
     */
    void ApplyRotation_PitchAxis(float pitch);
    
    /**
     * @brief Rotate this Orientation's local Quaternion by the specified Euler radians around the Roll axis only
     * adding it to the current rotation
     */
    void ApplyRotation_RollAxis(float roll);
    
    /**
     * @brief Set this Orientation's local Quaternion to the specified Euler radians Float3's equivalent Quaternion, this still incorporates recentered offsets
     */
    void SetRotation(Float3 rotationRadians);

    /**
     * @brief Set this Orientation's local Quaternion to the specified Quaternion, this still incorporates recentered offsets
     */
    void SetRotation(Quaternion quaternion);

    /**
     * @brief Set this Orientation's local Quaternion to the specified Euler radians Float3 equivalent Quaternion
     * and set the parent Quaternion to zero, resetting all offsets from recentering
     */
    void SetGlobalRotation(Float3 rotationRadians);

    /**
     * @brief Set this Orientation's local Quaternion to the specified Quaternion
     * and set the parent Quaternion to zero, resetting all offsets from recentering
     */
    void SetGlobalRotation(Quaternion quaternion);

    /**
     * @brief Set this Orientation's local and parent Quaternion to zero
     */
    void Reset();

    /**
     * @brief Set this Orientation's parent Quaternion to a value, such that the global rotation is zero
     */
    void Recenter();

private:
    Quaternion m_local;
    Quaternion m_parent;
};

} // namespace Mach1

#endif //M1_ORIENTATIONMANAGER_ORIENTATION_H
