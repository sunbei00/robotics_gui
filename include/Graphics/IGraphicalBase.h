//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_IGRAPHICALBASE_H
#define ROBOTICS_GUI_IGRAPHICALBASE_H
#include <QOpenGLFunctions_4_5_Core>
#include "glm.hpp"

namespace Graphics{
    void checkOpenGLError(const std::string &functionName);

    class IGraphicalBase{
    private:
    protected:
        virtual void genGL() = 0;
        virtual void delGL() = 0;
        GLuint compileShader(GLenum type, const char* source);
        GLuint compileProgram();
        void setMVPUniform(glm::mat4 viewMatrix, glm::mat4 projectionMatrix);
        virtual GLuint getProgram() = 0;
    protected:
        QOpenGLFunctions_4_5_Core* glFunc;
        const char* vertexShaderSource = nullptr;
        const char* fragmentShaderSource = nullptr;
        glm::mat4 modelMatrix = glm::mat4(1.f);
    public:
        explicit IGraphicalBase(QOpenGLFunctions_4_5_Core* glFunction);
        virtual ~IGraphicalBase();
        virtual void draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) = 0;
    };
}

#endif //ROBOTICS_GUI_IGRAPHICALBASE_H
