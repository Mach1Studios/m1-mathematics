#ifndef M1_ORIENTATIONMANAGER_QUATERNION_H
#define M1_ORIENTATIONMANAGER_QUATERNION_H

#include <string>

namespace Mach1 {

class Float3;

class Quaternion {
public:
    Quaternion();
    Quaternion(float qw, float qx, float qy, float qz);

    static Quaternion FromEulerDegrees(Float3 euler_vector);

    static Quaternion FromEulerRadians(Float3 euler_vector);

    Float3 ToEulerDegrees();

    Float3 ToEulerRadians();

    std::string ToString() const;

    bool IsApproximatelyEqual(const Quaternion &rhs) const;

    float DotProduct(Quaternion rhs) const;

    float Length() const;

    float LengthSquared() const;

    Quaternion Normalized() const;

    Quaternion Inversed() const;

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
