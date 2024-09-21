//
// Created by root on 9/13/24.
//

#ifndef ROBOTICS_GUI_TRIANGLERENDERER_H
#define ROBOTICS_GUI_TRIANGLERENDERER_H

#include "Graphics/IGraphicalBase.h"
#include "Graphics/OBJLoader.h"


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

namespace Graphics {
    class TriangleRenderer : public IGraphicalBase {
    private:
        static GLuint mProgram;
        GLuint vao = 0;
        GLuint vbo = 0;
        glm::vec3 mColor = glm::vec3(0.0941f, 0.6274f, 0.8431f);
        std::vector<glm::vec3> mData;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    public:
        explicit TriangleRenderer(const std::vector<glm::vec3>& data, QOpenGLFunctions_4_5_Core* glFunc);
        ~TriangleRenderer() override;
        void draw(const InteractionCamera& camera) override;
    };
}

#endif //ROBOTICS_GUI_TRIANGLERENDERER_H
