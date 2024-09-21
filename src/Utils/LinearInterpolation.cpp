//
// Created by root on 9/21/24.
//

#include "Utils/LinearInterpolation.h"

namespace Utils{
    glm::vec3 linearInterpolate(const glm::vec3& start, const glm::vec3& end, float t) {
        return start + t * (end - start);
    }

    std::vector<glm::vec3> interpolateBetweenPoints(const glm::vec3& start, const glm::vec3& end, float intervalCm) {
        std::vector<glm::vec3> interpolatedPositions;

        float distance = glm::distance(start, end);

        float distanceCm = distance * 100.0f;

        int numIntervals = static_cast<int>(distanceCm / intervalCm);

        for (int i = 0; i <= numIntervals; ++i) {
            float t = static_cast<float>(i) / numIntervals;
            glm::vec3 interpolatedPos = linearInterpolate(start, end, t);
            interpolatedPositions.push_back(interpolatedPos);
            interpolatedPositions.push_back(end);
        }

        return interpolatedPositions;
    }
}