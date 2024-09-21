//
// Created by root on 9/21/24.
//

#ifndef LINEARINTERPOLATION_H
#define LINEARINTERPOLATION_H
#include <vector>

#include "glm.hpp"

namespace Utils{
    glm::vec3 linearInterpolate(const glm::vec3& start, const glm::vec3& end, float t);

    std::vector<glm::vec3> interpolateBetweenPoints(const glm::vec3& start, const glm::vec3& end, float intervalCm);
}


#endif //LINEARINTERPOLATION_H
