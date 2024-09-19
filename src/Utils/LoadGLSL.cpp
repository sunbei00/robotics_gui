//
// Created by root on 9/19/24.
//
#include "Utils/LoadGLSL.h"

namespace Utils{
    std::string readGLSLFile(const std::string& filePath) {
        std::ifstream file(filePath);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filePath << std::endl;
            return nullptr;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();

        return buffer.str();
    }
}