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

namespace Graphics{

    void checkOpenGLError(const std::string &functionName) {
        GLenum error;
        while ((error = glGetError()) != GL_NO_ERROR) {
            std::string errorMessage;
            switch (error) {
                case GL_INVALID_OPERATION:
                    errorMessage = "Invalid operation";
                    break;
                case GL_INVALID_VALUE:
                    errorMessage = "Invalid value";
                    break;
                case GL_INVALID_ENUM:
                    errorMessage = "Invalid enum";
                    break;
                case GL_OUT_OF_MEMORY:
                    errorMessage = "Out of memory";
                    break;
                default:
                    errorMessage = "Unknown error";
                    break;
            }
            std::cerr << "OpenGL Error in " << functionName << ": " << errorMessage << std::endl;
        }
    }

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
        GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
        GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

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


