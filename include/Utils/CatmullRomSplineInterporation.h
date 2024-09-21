//
// Created by root on 9/21/24.
//

#ifndef CATMULLROMSPLINEINTERPORATION_H
#define CATMULLROMSPLINEINTERPORATION_H
#include <vector>
#include "glm.hpp"

namespace Utils{

    glm::vec3 catmullRom(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, float t);
    std::vector<glm::vec3> handleEdgeCases(const std::vector<glm::vec3>& vertices);
    std::vector<glm::vec3> sampleCatmullRomSpline(const std::vector<glm::vec3>& vertices, int samplesPerSegment);

}

#endif //CATMULLROMSPLINEINTERPORATION_H
