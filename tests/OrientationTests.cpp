#include <gtest/gtest.h>
#include <sstream>

#include "m1_mathematics/Orientation.h"

TEST(OrientationTests, Construction) {
    Mach1::Orientation zeroOri;
    ASSERT_TRUE(zeroOri.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(Mach1::Float3{}));
    ASSERT_TRUE(zeroOri.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(Mach1::Float3{}));
    ASSERT_TRUE(zeroOri.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(Mach1::Quaternion{}));
}

TEST(OrientationTests, Resetting) {

    Mach1::Orientation zeroOri;
    Mach1::Orientation nonZeroOri;
    nonZeroOri.ApplyRotationDegrees(Mach1::Float3{30, 45, -15});

    ASSERT_FALSE(zeroOri.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(nonZeroOri.GetGlobalRotationAsQuaternion()));
    ASSERT_FALSE(zeroOri.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(nonZeroOri.GetGlobalRotationAsEulerRadians()));
    ASSERT_FALSE(zeroOri.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(nonZeroOri.GetGlobalRotationAsEulerDegrees()));

    nonZeroOri.Reset();

    ASSERT_TRUE(zeroOri.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(nonZeroOri.GetGlobalRotationAsQuaternion()));
    ASSERT_TRUE(zeroOri.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(nonZeroOri.GetGlobalRotationAsEulerRadians()));
    ASSERT_TRUE(zeroOri.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(nonZeroOri.GetGlobalRotationAsEulerDegrees()));
}

TEST(OrientationTests, SettingRotation) {

    Mach1::Orientation ori;

    Mach1::Float3 rotDeg = Mach1::Float3{30, 45, -15};
    Mach1::Float3 rotRad = rotDeg.EulerRadians();
    Mach1::Quaternion rotQuat = Mach1::Quaternion::FromEulerDegrees(rotDeg);

    ori.SetGlobalRotation(rotQuat);

    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(rotRad));
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(rotDeg));
    ASSERT_TRUE(ori.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(rotQuat));

    ori.SetGlobalRotation(rotRad);

    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(rotRad));
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(rotDeg));
    ASSERT_TRUE(ori.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(rotQuat));

}

TEST(OrientationTests, RotationApplication) {

    using namespace Mach1;

    Orientation ori;

    Float3 testVec = Float3{45, 30, -10};
    Float3 testVecRads = testVec.EulerRadians();

    auto pitch = testVecRads[0];
    auto yaw = testVecRads[1];
    auto roll = testVecRads[2];

    Float3 euler_p(pitch, 0.0, 0.0);
    Float3 euler_y(0.0, yaw, 0.0);
    Float3 euler_r(0.0, 0.0, roll);
    Float3 euler_yp(pitch, yaw, 0.0);
    Float3 euler_yr(0.0, yaw, roll);
    Float3 euler_pr(pitch, 0.0, roll);
    Float3 euler_ypr(pitch, yaw, roll);

    Quaternion q_yp = Quaternion::FromEulerRadians(euler_yp);
    Quaternion q_yr = Quaternion::FromEulerRadians(euler_yr);
    Quaternion q_pr = Quaternion::FromEulerRadians(euler_pr);
    Quaternion q_ypr = Quaternion::FromEulerRadians(euler_ypr);

    ori.Reset();
    ori.ApplyRotation(euler_y);
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(euler_y));

    ori.Reset();
    ori.ApplyRotation(euler_p);
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(euler_p));

    ori.Reset();
    ori.ApplyRotation(euler_r);
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(euler_r));

    ori.Reset();
    ori.ApplyRotation(euler_y);
    ori.ApplyRotation(euler_p);
    ASSERT_TRUE(ori.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(q_yp));
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(euler_yp));

    ori.Reset();
    ori.ApplyRotation(euler_y);
    ori.ApplyRotation(euler_r);
    ASSERT_TRUE(ori.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(q_yr));
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(euler_yr));

    ori.Reset();
    ori.ApplyRotation(euler_p);
    ori.ApplyRotation(euler_r);
    ASSERT_TRUE(ori.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(q_pr));
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(euler_pr));

    ori.Reset();
    ori.ApplyRotation(euler_y);
    ori.ApplyRotation(euler_p);
    ori.ApplyRotation(euler_r);
    ASSERT_TRUE(ori.GetGlobalRotationAsQuaternion().IsApproximatelyEqual(q_ypr));
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(euler_ypr));

    ASSERT_TRUE(ori.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(testVec));
}

TEST(OrientationTests, Recentering) {

    using namespace Mach1;

    Orientation ori;
    Float3 zeroVec;

    ori.ApplyRotationDegrees(Float3{0, 10, 0});
    ori.Recenter();
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(zeroVec));
    ori.SetRotation(zeroVec);
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(Float3{0, -10, 0}));
    ori.ApplyRotationDegrees({Float3{0, -10, 0}});

    ori.Reset();

    ori.ApplyRotationDegrees(Float3{45, 0, 0});
    ori.Recenter();
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(zeroVec));
    ori.SetRotation(zeroVec);
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(Float3{-45, 0, 0}));
    ori.ApplyRotationDegrees({Float3{-45, 0, 0}});

    ori.Reset();

    ori.ApplyRotationDegrees(Float3{0, 0, -10});
    ori.Recenter();
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerRadians().IsApproximatelyEqual(zeroVec));
    ori.SetRotation(zeroVec);
    ASSERT_TRUE(ori.GetGlobalRotationAsEulerDegrees().IsApproximatelyEqual(Float3{0, 0, 10}));
    ori.ApplyRotationDegrees({Float3{0, 0, 10}});
}