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

#endif //ROBOTICS_GUI_TRIANGLERENDERER_H
