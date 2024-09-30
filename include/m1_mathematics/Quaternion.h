#ifndef M1_ORIENTATIONMANAGER_QUATERNION_H
#define M1_ORIENTATIONMANAGER_QUATERNION_H

#include <string>

namespace Mach1 {

class Float3;

class Quaternion {
public:
    Quaternion();
    Quaternion(float qw, float qx, float qy, float qz);

    /**
     * @brief Construct a Quaternion from a given Euler degrees Float3
     * @param euler_vector Float3, whose components are rotations around respective axes in degrees
     * @return corresponding Quaternion
     */
    static Quaternion FromEulerDegrees(Float3 euler_degrees);

    /**
     * @brief Construct a Quaternion from a given Euler radians Float3
     * @param euler_vector Float3, whose components are rotations around respective axes in radians
     * @return corresponding Quaternion
     */
    static Quaternion FromEulerRadians(Float3 euler_radians);

    /**
     * @brief Construct a Euler degrees Float3 from this Quaternion
     * @return Float3, whose components are rotations in degrees around corresponding axes
     */
    Float3 ToEulerDegrees();

    /**
     * @brief Construct a Euler radians Float3 from this Quaternion
     * @return Float3, whose components are rotations in radians around corresponding axes
     */
    Float3 ToEulerRadians();

    /**
     * @brief Check whether this Quaternion is equal to the given Quaternion within a margin of error
     */
    bool IsApproximatelyEqual(const Quaternion &rhs) const;

    /**
     * @brief Get the standard Euclidean 4D dot product for this Quaternion and the given Quaternion
     */
    float DotProduct(Quaternion rhs) const;

    /**
     * @brief Get the length of this Quaternion
     */
    float Length() const;

    /**
     * @brief Get the squared length of this Quaternion (dot product with itself)
     */
    float LengthSquared() const;

    /**
     * @brief Get this Quaternion, divided by its own length
     */
    Quaternion Normalized() const;

    /**
     * @brief Get a Quaternion, such that it multiplied by this Quaternion would result in a zero Quaternion
     */
    Quaternion Inversed() const;

    /**
     * @brief Get the string representation of this Quaternion
     * @return string of the format "Quaternion(w: `w`, x: `x`, y: `y`, z: `z`"
     */
    std::string ToString() const;

    /**
     * @brief Get the W
     */
    float GetW() const;
    
    /**
     * @brief Get the X
     */
    float GetX() const;
    
    /**
     * @brief Get the Y
     */
    float GetY() const;

    /**
     * @brief Get the Z value
     */
    float GetZ() const;

    void operator*=(float scalar);
    void operator/=(float scalar);
    void operator*=(const Quaternion &rhs);

    bool operator==(const Quaternion& rhs) const;
    bool operator!=(const Quaternion& rhs) const;

    Quaternion operator*(float scalar) const;
    Quaternion operator/(float scalar) const;
    Quaternion operator*(const Quaternion &rhs) const;
    Quaternion operator+(const Quaternion &rhs) const;
    Quaternion operator-(const Quaternion &rhs) const;

    const float &operator[](int axis) const;
    float &operator[](int axis);

private:
    float m_qw;
    float m_qx;
    float m_qy;
    float m_qz;
};

} // namespace Mach1

#endif //M1_ORIENTATIONMANAGER_QUATERNION_H
