#ifndef M1_ORIENTATIONMANAGER_MATHUTILITY_H
#define M1_ORIENTATIONMANAGER_MATHUTILITY_H

#include <cmath>

namespace Mach1 {

class MathUtility {
public:

    static constexpr float FLOAT_COMPARISON_EPSILON = 0.00001;
    static constexpr float FLOAT_COMPARISON_ONE_MINUS_EPSILON = 1 - FLOAT_COMPARISON_EPSILON;

    static bool IsApproximatelyEqual(float a, float b) {

        if (a == b) {
            return true;
        }

        float tolerance = FLOAT_COMPARISON_EPSILON * abs(a);
        if (tolerance < FLOAT_COMPARISON_EPSILON) {
            tolerance = FLOAT_COMPARISON_EPSILON;
        }

        return abs(a - b) < tolerance;
    }
};

} // namespace Mach1

#endif //M1_ORIENTATIONMANAGER_MATHUTILITY_H
