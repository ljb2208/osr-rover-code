#ifndef HELPER_H
#define HELPER_H

#include <cmath>
#include <algorithm>

namespace OsrPlanner {
    namespace Helper {

        static inline float normalizeHeadingRad(float t) {
            if (t < 0) {
                t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
                return 2.f * M_PI + t;
            }

            return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
        }
        
        static inline float toDeg(float t) {
            return normalizeHeadingRad(t) * 180.f / M_PI ;
        }

        static inline float clamp(float n, float lower, float upper) {
            return std::max(lower, std::min(n, upper));
        }
    }
}

#endif // HELPER_H