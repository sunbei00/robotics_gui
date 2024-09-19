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
        virtual void draw(const InteractionCamera& camera) = 0;
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
        GLuint vao = 0;
        std::vector<GLuint> vbo;

        pcd_data mData;

    public:
        explicit PointRendererSeparated(QOpenGLFunctions_4_5_Core* glFunc, const pcd_data& data);
        explicit PointRendererSeparated(QOpenGLFunctions_4_5_Core* glFunc, pcd_data&& data);
        ~PointRendererSeparated() override;
        void draw(const InteractionCamera& camera) override;
    };
}

namespace Graphics{
    class PointRendererSeparatedFiltered : public IPointRenderer{
    private:
        static GLuint mProgram;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    protected:
        GLuint vao = 0;
        std::vector<GLuint> vbo;
        pcd_data mData;

    public:
        explicit PointRendererSeparatedFiltered(QOpenGLFunctions_4_5_Core* glFunc, const pcd_data& data);
        explicit PointRendererSeparatedFiltered(QOpenGLFunctions_4_5_Core* glFunc, pcd_data&& data);
        ~PointRendererSeparatedFiltered() override;
        void draw(const InteractionCamera& camera) override;
    };
}

namespace Graphics{
    // To do : Test Code, Only Implementation
    class PointRendererInterleaved : public IPointRenderer{
    private:
        static GLuint mProgram;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    protected:
        GLuint vao = 0;
        GLuint vbo = 0;

        std::vector<glm::vec3> mData;
    public:
        explicit PointRendererInterleaved(QOpenGLFunctions_4_5_Core* glFunc, const std::vector<glm::vec3>& data);
        explicit PointRendererInterleaved(QOpenGLFunctions_4_5_Core* glFunc, std::vector<glm::vec3>&& data);
        ~PointRendererInterleaved() override;
        void draw(const InteractionCamera& camera) override;
    };
}

namespace Graphics{
    // To do : Test Code, Only Implementation
    class PointRendererInterleavedFiltered : public IPointRenderer{
    private:
        static GLuint mProgram;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    protected:
        GLuint vao = 0;
        GLuint vbo = 0;

        std::vector<glm::vec3> mData;
    public:
        explicit PointRendererInterleavedFiltered(QOpenGLFunctions_4_5_Core* glFunc, const std::vector<glm::vec3>& data);
        explicit PointRendererInterleavedFiltered(QOpenGLFunctions_4_5_Core* glFunc, std::vector<glm::vec3>&& data);
        ~PointRendererInterleavedFiltered() override;
        void draw(const InteractionCamera& camera) override;
    };
}

#endif //ROBOTICS_GUI_POINTRENDERER_H
