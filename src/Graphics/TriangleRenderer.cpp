//
// Created by root on 9/13/24.
//


#include "Graphics/TriangleRenderer.h"


namespace Graphics{

    GLuint TriangleRenderer::mProgram = 0;

    TriangleRenderer::TriangleRenderer(QOpenGLFunctions_4_5_Core *glFunc) : IGraphicalBase(glFunc){
        genGL();
    }

    TriangleRenderer::~TriangleRenderer(){
        delGL();
    }

    void TriangleRenderer::delGL(){
        if(vao != 0)
            glFunc->glDeleteVertexArrays(1, &vao);
        if(vbo != 0)
            glFunc->glDeleteBuffers(1, &vbo);
        vao = 0;
        vbo = 0;
    }

    void TriangleRenderer::genGL() {
        delGL();
        glFunc->glGenVertexArrays(1, &vao);
        glFunc->glBindVertexArray(vao);
        glFunc->glGenBuffers(1, &vbo);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glFunc->glBindVertexArray(0);
    }

    void TriangleRenderer::draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        glFunc->glUseProgram(getProgram());
        setMVPUniform(viewMatrix, projectionMatrix);

    }

    GLuint TriangleRenderer::getProgram() {
        if(mProgram == 0){
            vertexShaderSource = Utils::readGLSLFile(shaderPath + "/triangleVertexShaderSource.glsl");
            fragmentShaderSource = Utils::readGLSLFile(shaderPath + "/triangleFragmentShaderSource.glsl");
            mProgram = compileProgram();
        }
        return mProgram;
    }
}
