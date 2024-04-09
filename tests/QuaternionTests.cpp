#include <gtest/gtest.h>
#include <sstream>

#include "m1_mathematics/Quaternion.h"
#include "m1_mathematics/Float3.h"

TEST(QuaternionTests, Construction) {
    Mach1::Quaternion zeroQuat = {};
    ASSERT_FLOAT_EQ(zeroQuat[0], 1); // qw
    ASSERT_FLOAT_EQ(zeroQuat[1], 0); // qx
    ASSERT_FLOAT_EQ(zeroQuat[2], 0); // qy
    ASSERT_FLOAT_EQ(zeroQuat[3], 0); // qz

    Mach1::Quaternion weirdQuat = {1, 2, 3, 4};
    ASSERT_FLOAT_EQ(weirdQuat[0], 1); // qw
    ASSERT_FLOAT_EQ(weirdQuat[1], 2); // qx
    ASSERT_FLOAT_EQ(weirdQuat[2], 3); // qy
    ASSERT_FLOAT_EQ(weirdQuat[3], 4); // qz

    ASSERT_NE(zeroQuat, weirdQuat);
}

TEST(QuaternionTests, EulerToQuatConversions) {
    std::pair<Mach1::Float3, Mach1::Quaternion> conversionTestCases[] = {
            {{0, 0, 0},   {1.0, 0.0, 0.0, 0.0}},
            {{0, 45, 0},   {0.923879, 0.0, 0.382684, 0.0}},
            {{30, 0, 0},   {0.965926, 0.258819, 0.0, 0.0}},
            {{0, 0, 10},   {0.996195, 0, 0.0, 0.0871558}},
    };

    int conversionFailureCount = 0;
    std::stringstream potentialErrorMessage;
    for (int i = 0; i < size(conversionTestCases); i++) {

        auto v = conversionTestCases[i].first;
        auto q = conversionTestCases[i].second;

        auto convQuat = Mach1::Quaternion::FromEulerDegrees(v);
        if (q.IsApproximatelyEqual(convQuat)) continue;

        conversionFailureCount++;
        potentialErrorMessage << "\n" << v.ToString() << " -> Actual " << convQuat.ToString();
        potentialErrorMessage << "; Expected " << q.ToString();
    }

    ASSERT_EQ(conversionFailureCount, 0) << potentialErrorMessage.str();
}

TEST(QuaternionTests, Inverse) {
    using namespace Mach1;

    Float3 zeroVec = {};
    Quaternion zeroQuat = {};
    ASSERT_EQ(zeroQuat, zeroQuat.Inversed());

    Float3 testVec = {45, 30, 10};
    Quaternion testQuat = Quaternion::FromEulerRadians(testVec.EulerRadians());
    Quaternion reverseTestQuat = testQuat.Inversed();
    ASSERT_TRUE(zeroQuat.IsApproximatelyEqual(testQuat * reverseTestQuat));
    ASSERT_TRUE(zeroVec.IsApproximatelyEqual((testQuat * reverseTestQuat).ToEulerDegrees()));
}

TEST(QuaternionTests, Multiplication) {
    using namespace Mach1;

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

    Quaternion q_p = Quaternion::FromEulerRadians(euler_p);
    Quaternion q_y = Quaternion::FromEulerRadians(euler_y);
    Quaternion q_r = Quaternion::FromEulerRadians(euler_r);
    Quaternion q_yp = Quaternion::FromEulerRadians(euler_yp);
    Quaternion q_yr = Quaternion::FromEulerRadians(euler_yr);
    Quaternion q_pr = Quaternion::FromEulerRadians(euler_pr);
    Quaternion q_ypr = Quaternion::FromEulerRadians(euler_ypr);

    Float3 conv_p = q_p.ToEulerRadians();
    Float3 conv_y = q_y.ToEulerRadians();
    Float3 conv_r = q_r.ToEulerRadians();
    Float3 conv_yp = q_yp.ToEulerRadians();
    Float3 conv_yr = q_yr.ToEulerRadians();
    Float3 conv_pr = q_pr.ToEulerRadians();
    Float3 conv_ypr = q_ypr.ToEulerRadians();

    Float3 convTestVec = conv_ypr.EulerDegrees();

    ASSERT_TRUE(conv_p.IsApproximatelyEqual(euler_p)) << euler_p.ToString() << " != " << conv_p.ToString();
    ASSERT_TRUE(conv_y.IsApproximatelyEqual(euler_y)) << euler_y.ToString() << " != " << conv_y.ToString();
    ASSERT_TRUE(conv_r.IsApproximatelyEqual(euler_r)) << euler_r.ToString() << " != " << conv_r.ToString();

    ASSERT_TRUE(q_yp.IsApproximatelyEqual(q_y * q_p));
    ASSERT_TRUE(conv_yp.IsApproximatelyEqual(euler_yp)) << euler_yp.ToString() << " != " << conv_yp.ToString();

    ASSERT_TRUE(q_yr.IsApproximatelyEqual(q_y * q_r));
    ASSERT_TRUE(conv_yr.IsApproximatelyEqual(euler_yr)) << euler_yr.ToString() << " != " << conv_yr.ToString();

    ASSERT_TRUE(q_pr.IsApproximatelyEqual(q_p * q_r));
    ASSERT_TRUE(conv_pr.IsApproximatelyEqual(euler_pr)) << euler_pr.ToString() << " != " << conv_pr.ToString();

    ASSERT_TRUE(q_ypr.IsApproximatelyEqual(q_y * q_p * q_r));
    ASSERT_TRUE(conv_ypr.IsApproximatelyEqual(euler_ypr)) << euler_ypr.ToString() << " != " << conv_ypr.ToString();

    ASSERT_TRUE(convTestVec.IsApproximatelyEqual(testVec)) << convTestVec.ToString() << " != " << testVec.ToString();
}