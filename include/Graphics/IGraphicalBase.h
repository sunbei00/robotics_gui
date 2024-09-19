//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_IGRAPHICALBASE_H
#define ROBOTICS_GUI_IGRAPHICALBASE_H
#include <QOpenGLFunctions_4_5_Core>
#include "Graphics/Camera.h"
#include "Utils/LoadGLSL.h"
#include "glm.hpp"

namespace Graphics{
    struct ZFilter{
        static float mZMin;
        static float mZMax;

        static float mLimitZMin;
        static float mLimitZMax;
    };
}

namespace Graphics{
    struct Light {
        static glm::vec3 direction;
        static glm::vec3 ambient;
        static glm::vec3 diffuse;
        static glm::vec3 specular;
    };
}

namespace Graphics{

    class IGraphicalBase{
    protected:
        virtual void genGL() = 0;
        virtual void delGL() = 0;
        GLuint compileShader(GLenum type, const char* source);
        GLuint compileProgram();
        void setMVPUniform(glm::mat4 viewMatrix, glm::mat4 projectionMatrix);
        virtual GLuint getProgram() = 0;
    protected:
        QOpenGLFunctions_4_5_Core* glFunc;
        std::string vertexShaderSource;
        std::string fragmentShaderSource;
        static std::string shaderPath;
    public:
        glm::mat4 modelMatrix = glm::mat4(1.f);
        explicit IGraphicalBase(QOpenGLFunctions_4_5_Core* glFunction);
        virtual ~IGraphicalBase();
        virtual void draw(const InteractionCamera& camera) = 0;
    };
}

#endif //ROBOTICS_GUI_IGRAPHICALBASE_H
