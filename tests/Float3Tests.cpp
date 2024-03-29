#include <gtest/gtest.h>

#include "m1_mathematics/Float3.h"

TEST(Float3Tests, DefaultConstructor) {
    Mach1::Float3 zeroVec = {};
    ASSERT_FLOAT_EQ(zeroVec[0], 0);
    ASSERT_FLOAT_EQ(zeroVec[1], 0);
    ASSERT_FLOAT_EQ(zeroVec[2], 0);
}

TEST(Float3Tests, ComponentWiseConstructor) {
    Mach1::Float3 standardConstructorVec = {0, 0.5, 1};
    ASSERT_FLOAT_EQ(standardConstructorVec[0], 0);
    ASSERT_FLOAT_EQ(standardConstructorVec[1], 0.5);
    ASSERT_FLOAT_EQ(standardConstructorVec[2], 1);
}

TEST(Float3Tests, ScalarConstructor) {

    Mach1::Float3 scalarConstructorVec = {M_PI};
    ASSERT_FLOAT_EQ(scalarConstructorVec[0], M_PI);
    ASSERT_FLOAT_EQ(scalarConstructorVec[1], M_PI);
    ASSERT_FLOAT_EQ(scalarConstructorVec[2], M_PI);
}

TEST(Float3Tests, NormalizationOfZeroVector) {
    Mach1::Float3 zeroVec = {};
    ASSERT_FLOAT_EQ(zeroVec.Length(), 0.0);
    ASSERT_FLOAT_EQ(zeroVec.Normalized().Length(), 0.0);
}

TEST(Float3Tests, NormalizationOfVector) {
    Mach1::Float3 almostZeroVec = {0.0, 0.0000123456, 0.0};

    ASSERT_NE(almostZeroVec.Length(), 0.0);
    ASSERT_FLOAT_EQ(almostZeroVec.Normalized().Length(), 1.0);

    Mach1::Float3 upVec = {0.0, 1.0, 0.0};
    ASSERT_FLOAT_EQ(upVec.Length(), 1.0);
    ASSERT_FLOAT_EQ(upVec.Normalized().Length(), 1.0);

    ASSERT_EQ(almostZeroVec.Normalized(), upVec);
}

TEST(Float3Tests, Clamp) {
    Mach1::Float3 zeroVec;
    Mach1::Float3 fiveVec = {5.0};
    Mach1::Float3 minTenVec = {-10};
    Mach1::Float3 tenVec = {10};
    Mach1::Float3 vec = {1, 2, 3};

    ASSERT_EQ(fiveVec.Clamped(fiveVec, fiveVec), fiveVec);
    ASSERT_EQ(fiveVec.Clamped(minTenVec, fiveVec), fiveVec);
    ASSERT_EQ(fiveVec.Clamped(minTenVec, vec), vec);
    ASSERT_EQ(fiveVec.Clamped(minTenVec, minTenVec), minTenVec);
    ASSERT_EQ(fiveVec.Clamped(tenVec, tenVec), tenVec);
    ASSERT_EQ(zeroVec.Clamped(vec, tenVec), vec);
    ASSERT_EQ(zeroVec.Clamped(tenVec, vec), tenVec);
}

TEST(Float3Tests, EulerConversion) {

    Mach1::Float3 radianVec = {M_PI, M_PI_4, M_PI_2};
    Mach1::Float3 degreeVec = radianVec.EulerDegrees();

    Mach1::Float3 degreeVec2 = {180, 45, 90};
    Mach1::Float3 radianVec2 = degreeVec2.EulerRadians();

    ASSERT_EQ(radianVec, radianVec2);
    ASSERT_EQ(degreeVec, degreeVec2);
    ASSERT_EQ(radianVec.EulerDegrees(), degreeVec2);
    ASSERT_EQ(radianVec2.EulerDegrees(), degreeVec);
    ASSERT_EQ(degreeVec.EulerRadians(), radianVec);
    ASSERT_EQ(degreeVec.EulerRadians(), radianVec2);
    ASSERT_EQ(degreeVec2.EulerRadians(), radianVec);
    ASSERT_EQ(degreeVec2.EulerRadians(), radianVec2);
}

TEST(Float3Tests, DenormalizationOfZeroVector) {
    using namespace Mach1;

    Float3 zeroVec = {};
    Float3 pointFiveVec = {0.5};
    ASSERT_EQ(pointFiveVec.Map(0, 1, -1, 1), zeroVec);
    ASSERT_EQ(zeroVec.Map(-1, 1, 0, 1), pointFiveVec);
    ASSERT_EQ(zeroVec.Map(-1, 1, -M_PI, M_PI), zeroVec);
}

TEST(Float3Tests, DenormalizationOfEulerRotationVector) {
    using namespace Mach1;

    Float3 angleVec = {30, -45, 90};
    Float3 convertedAngleVec = angleVec.EulerRadians();
    Float3 shiftedAngleVec = angleVec + 180;

    Float3 denormalizedAngleVec = angleVec.Map(-180, 180, -M_PI, M_PI);
    Float3 denormalizedAngleVec2 = angleVec.Map(-180, 180, 0, M_PI * 2);

    ASSERT_TRUE(denormalizedAngleVec.IsApproximatelyEqual(convertedAngleVec));
    ASSERT_TRUE(denormalizedAngleVec2.IsApproximatelyEqual(shiftedAngleVec.EulerRadians()));

    Float3 zeroVec = {};
    Float3 piVec = {M_PI};
    Float3 shiftedVec = zeroVec.Map(-M_PI, M_PI, 0, M_PI * 2.0);
    ASSERT_EQ(shiftedVec, piVec);

}

TEST(Float3Tests, DenormalizationOfStandardVector) {
    using namespace Mach1;

    Float3 zeroVec = {};

    Float3 twoHundoVec = {200};
    Float3 denormVec = zeroVec.Map(-1, 1, 150, 250);
    ASSERT_EQ(denormVec, twoHundoVec);
}
