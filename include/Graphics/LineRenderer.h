//
// Created by root on 9/13/24.
//

#ifndef ROBOTICS_GUI_LINERENDERER_H
#define ROBOTICS_GUI_LINERENDERER_H
#include "Graphics/IGraphicalBase.h"

// To do : Implementation

namespace Graphics{

    class LineRenderer : public IGraphicalBase{
    private:
        static GLuint mProgram;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    protected:
        GLuint vao;
        GLuint vbo;
    public:
        explicit LineRenderer(QOpenGLFunctions_4_5_Core* glFunc);
        ~LineRenderer() override;
        void draw(glm::mat4 viewMatrix = glm::mat4(1.f), glm::mat4 projectionMatrix = glm::mat4(1.f)) override;

    };
}

#endif //ROBOTICS_GUI_LINERENDERER_H
