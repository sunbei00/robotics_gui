//
// Created by root on 9/13/24.
//

#ifndef ROBOTICS_GUI_TRIANGLERENDERER_H
#define ROBOTICS_GUI_TRIANGLERENDERER_H

#include "Graphics/IGraphicalBase.h"
#include "Graphics/OBJLoader.h"

// To do : Implementation

namespace Graphics{
    class TriangleRenderer : public IGraphicalBase{
    private:
        static GLuint mProgram;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    protected:
        GLuint vao = 0;
        GLuint vbo = 0;
    public:
        explicit TriangleRenderer(QOpenGLFunctions_4_5_Core* glFunc);
        ~TriangleRenderer() override;
        void draw(const InteractionCamera& camera) override;
    };
}

namespace Graphics{
    class OBJLoaderTriangleRenderer : public IGraphicalBase{
    private:
        static GLuint mProgram;
        OBJLoader* objLoader = nullptr;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    public:
        explicit OBJLoaderTriangleRenderer(const std::string& path, QOpenGLFunctions_4_5_Core* glFunc);
        ~OBJLoaderTriangleRenderer() override;
        void draw(const InteractionCamera& camera) override;
    };
}

#endif //ROBOTICS_GUI_TRIANGLERENDERER_H
