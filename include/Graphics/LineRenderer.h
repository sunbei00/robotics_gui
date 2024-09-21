//
// Created by root on 9/21/24.
//

#ifndef LINERENDERER_H
#define LINERENDERER_H
#include "IGraphicalBase.h"

namespace Graphics{
    class LineRenderer : public IGraphicalBase{
    private:
        std::string geometryShaderSource;
    private:
        static GLuint mProgram;
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint ebo = 0;
        glm::vec3 mColor = glm::vec3(0.0f,0.74509f , 0.22352f);
        std::vector<glm::vec3> mData;
    protected:
        GLuint getProgram() override;
        void genGL() override;
        void delGL() override;
    public:
        explicit LineRenderer(const std::vector<glm::vec3>& data, QOpenGLFunctions_4_5_Core* glFunc);
        ~LineRenderer() override;
        void draw(const InteractionCamera& camera) override;
    };
}


#endif //LINERENDERER_H
