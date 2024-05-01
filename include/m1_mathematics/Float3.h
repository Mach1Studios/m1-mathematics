#ifndef M1_ORIENTATIONMANAGER_FLOAT3_H
#define M1_ORIENTATIONMANAGER_FLOAT3_H

#include <string>

namespace Mach1 {

class Float3 {
public:
    Float3();
    Float3(float component);
    Float3(float x, float y, float z);

    /**
     * @brief Get the length of this Float3
     * @return the square root of the sum of the squares of this Float3's components
     */
    float Length() const;

    /**
     * @brief Return a Float3 with the same direction as this Float3, but with a length of 1
     * @return Float3, whose components are this Float3's components divided by its length
     */
    Float3 Normalized() const;

    /**
     * @brief Assuming this is a Float3 of radians, create a corresponding Float3 of degrees
     * @return Float3, where components are rotations in degrees around X, Y and Z axes respectively
     */
    Float3 EulerDegrees() const;

    /**
     * @brief Assuming this is a Float3 of degrees, create a corresponding Float3 of radians
     * @return Float3, where components are rotations in radians around X, Y and Z axes respectively
     */
    Float3 EulerRadians() const;

    /**
     * @brief Create a Float3, whose components are clamped between the components of the given Float3 instances
     * @return Float3, whose components are <= those of max and >= those of min
     */
    Float3 Clamped(Float3 min, Float3 max) const;

    /**
     * @brief Create a Float3, whose components are this Float3's components, proportionally remapped from the
     * input range to the given output range.
     */
    Float3 Map(float from_min, float from_max, float to_min, float to_max);

    /**
     * @brief Check whether this Float3 is equal to the given Float3 within a margin of error
     */
    bool IsApproximatelyEqual(const Float3 &rhs) const;

    /**
     * @brief Get the string representation of this Float3
     * @return string of the format "Float3(`x-component`, `y-component`, `z-component`)"
     */
    std::string ToString() const;
    
    /**
     * @brief Get the Yaw value where yaw is a right handed rotation around the Z-axis.
     *  Lowest value rotates to the right and Highest value rotates to the left
     */
    float GetYaw() const;
    
    /**
     * @brief Get the Pitch value where pitch is a downward rotation around the Y-axis.
     *  Lowest value rotates upward and Highest value rotates downward
     */
    float GetPitch() const;
    
    /**
     * @brief Get the Roll value where roll is a right handed rotation around the X-axis.
     *  Lowest value rotates to the right and Highest value rotates to the left
     */
    float GetRoll() const;

    const float &operator[](int axis) const;
    float &operator[](int axis);

    bool operator==(const Float3& rhs) const;
    bool operator!=(const Float3& rhs) const;

    Float3 &operator+=(const Float3 &rhs);
    Float3 &operator-=(const Float3 &rhs);
    Float3 &operator*=(const Float3 &rhs);
    Float3 &operator/=(const Float3 &rhs);

    Float3 &operator*=(float rhs_scalar);
    Float3 &operator/=(float rhs_scalar);

    Float3 operator+(const Float3 &rhs) const;
    Float3 operator-(const Float3 &rhs) const;
    Float3 operator*(const Float3 &rhs) const;
    Float3 operator/(const Float3 &rhs) const;

    Float3 operator+(float scalar) const;
    Float3 operator-(float scalar) const;
    Float3 operator*(float scalar) const;
    Float3 operator/(float scalar) const;

private:
    float m_x;
    float m_y;
    float m_z;
};

} // namespace Mach1


#endif //M1_ORIENTATIONMANAGER_FLOAT3_H
