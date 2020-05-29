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

        static inline float euclidianDistance(float x1, float y1, float x2, float y2)
        {
            return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
        }

        static inline float minVal(float v1, float v2, float v3, float v4)
        {
            float val = std::min(v1, v2);
            val = std::min(val, v3);
            return std::min(val, v4);
        }

        static inline float maxVal(float v1, float v2, float v3, float v4)
        {
            float val = std::max(v1, v2);
            val = std::max(val, v3);
            return std::max(val, v4);
        }
        
    }
}

#endif // HELPER_H