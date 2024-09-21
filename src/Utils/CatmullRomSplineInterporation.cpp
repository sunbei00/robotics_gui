//
// Created by root on 9/21/24.
//
#include "Utils/CatmullRomSplineInterporation.h"

namespace Utils {
    glm::vec3 catmullRom(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, float t) {
        float t2 = t * t;
        float t3 = t2 * t;

        return 0.5f * ((2.0f * p1) +
                       (-p0 + p2) * t +
                       (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                       (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
    }

    std::vector<glm::vec3> handleEdgeCases(const std::vector<glm::vec3>& vertices) {
        std::vector<glm::vec3> extendedVertices = vertices;

        assert(vertices.size() >= 2);

        if (vertices.size() == 2) {
            glm::vec3 p0 = vertices[0];
            glm::vec3 p3 = vertices[1];

            glm::vec3 p1 = p0 + (p3 - p0) * 0.33f;
            glm::vec3 p2 = p0 + (p3 - p0) * 0.66f;

            extendedVertices = { p0, p1, p2, p3 };
        } else if (vertices.size() == 3) {
            glm::vec3 p0 = vertices[0];
            glm::vec3 p1 = vertices[1];
            glm::vec3 p3 = vertices[2];
            glm::vec3 p2 = p1 + (p3 - p1) * 0.5f;

            extendedVertices = { p0, p1, p2, p3 };
        }

        return extendedVertices;
    }

    std::vector<glm::vec3> sampleCatmullRomSpline(const std::vector<glm::vec3>& vertices, int samplesPerSegment) {
        std::vector<glm::vec3> result;

        std::vector<glm::vec3> extendedVertices = handleEdgeCases(vertices);

        if (extendedVertices.size() >= 4) {
            for (size_t i = 1; i < extendedVertices.size() - 2; ++i) {
                for (int j = 0; j < samplesPerSegment; ++j) {
                    float t = j / static_cast<float>(samplesPerSegment);
                    glm::vec3 point = catmullRom(extendedVertices[i - 1], extendedVertices[i], extendedVertices[i + 1], extendedVertices[i + 2], t);
                    result.push_back(point);
                }
            }
            result.push_back(extendedVertices.back());
        }

        return result;
    }

}