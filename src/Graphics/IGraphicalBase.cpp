//
// Created by root on 9/13/24.
//

#include "Graphics/IGraphicalBase.h"
#include "gtc/type_ptr.hpp"
#include <iostream>



namespace Graphics{
    float ZFilter::mZMin = -30;
    float ZFilter::mZMax = 100;

    float ZFilter::mLimitZMin = -30;
    float ZFilter::mLimitZMax = 100;
}

namespace Graphics {
glm::vec3 Light::direction{0.0f, -1.0f, -1.0f};
    glm::vec3 Light::ambient{0.35f, 0.35f, 0.35f};
    glm::vec3 Light::diffuse{0.8f, 0.8f, 0.8f};
    glm::vec3 Light::specular{1.0f, 1.0f, 1.0f};
}

namespace Graphics{


    std::string IGraphicalBase::shaderPath = std::string(SHADER_DIR);

    IGraphicalBase::IGraphicalBase(QOpenGLFunctions_4_5_Core *glFunction) : glFunc(glFunction) {}

    IGraphicalBase::~IGraphicalBase() = default;

    GLuint IGraphicalBase::compileShader(GLenum type, const char* source){
        GLuint shader = glFunc->glCreateShader(type);
        glFunc->glShaderSource(shader, 1, &source, nullptr);
        glFunc->glCompileShader(shader);

        GLint success;
        glFunc->glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glFunc->glGetShaderInfoLog(shader, 512, nullptr, infoLog);
            std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
        }
        return shader;
    }

    GLuint IGraphicalBase::compileProgram(){
        GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource.c_str());
        GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource.c_str());

        GLuint shaderProgram = glFunc->glCreateProgram();
        glFunc->glAttachShader(shaderProgram, vertexShader);
        glFunc->glAttachShader(shaderProgram, fragmentShader);
        glFunc->glLinkProgram(shaderProgram);

        GLint success;
        glFunc->glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glFunc->glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
            std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        }

        glFunc->glDeleteShader(vertexShader);
        glFunc->glDeleteShader(fragmentShader);
        return shaderProgram;
    }

    void IGraphicalBase::setMVPUniform(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        GLuint program = getProgram();
        glFunc->glUseProgram(program);

        GLint modelLoc = glFunc->glGetUniformLocation(program, "model");
        glFunc->glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));

        GLint viewLoc = glFunc->glGetUniformLocation(program, "view");
        glFunc->glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));

        GLint projectionLoc = glFunc->glGetUniformLocation(program, "projection");
        glFunc->glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
    }
}


