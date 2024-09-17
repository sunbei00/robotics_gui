//
// Created by root on 9/13/24.
//

#ifndef ROBOTICS_GUI_POINTRENDERER_H
#define ROBOTICS_GUI_POINTRENDERER_H
#include "Graphics/IGraphicalBase.h"
#include <map>



namespace Graphics{
    using pcd_data =  std::map<std::string, std::vector<float>>;


    class IPointRenderer : public IGraphicalBase{
    protected:
        virtual GLuint getProgram() = 0;
        virtual void genGL() = 0;
        virtual void delGL() = 0;
    protected:
        glm::vec3 mColor = {1.0,1.0,1.0};
    public:
        explicit IPointRenderer(QOpenGLFunctions_4_5_Core* glFunc);
        virtual ~IPointRenderer();
        virtual void draw(glm::mat4 viewMatrix = glm::mat4(1.f), glm::mat4 projectionMatrix = glm::mat4(1.f)) = 0;
    };
}

namespace Graphics{
    class PointRendererSeparated : public IPointRenderer{
    private:
        static GLuint mProgram;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    protected:
        GLuint vao;
        std::vector<GLuint> vbo;

        pcd_data mData;

    public:
        explicit PointRendererSeparated(QOpenGLFunctions_4_5_Core* glFunc, const pcd_data& data);
        explicit PointRendererSeparated(QOpenGLFunctions_4_5_Core* glFunc, pcd_data&& data);
        ~PointRendererSeparated() override;
        void draw(glm::mat4 viewMatrix = glm::mat4(1.f), glm::mat4 projectionMatrix = glm::mat4(1.f)) override;
    };
}

namespace Graphics{
    class PointRendererInterleaved : public IPointRenderer{
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
        explicit PointRendererInterleaved(QOpenGLFunctions_4_5_Core* glFunc);
        ~PointRendererInterleaved() override;
        void draw(glm::mat4 viewMatrix = glm::mat4(1.f), glm::mat4 projectionMatrix = glm::mat4(1.f)) override;
    };
}

#endif //ROBOTICS_GUI_POINTRENDERER_H
