//
// Created by root on 9/13/24.
//

#include "Graphics/LineRenderer.h"

namespace Graphics{

    GLuint LineRenderer::mProgram = 0;

    LineRenderer::LineRenderer(QOpenGLFunctions_4_5_Core *glFunc) : IGraphicalBase(glFunc){
        genGL();
    }

    LineRenderer::~LineRenderer(){
        delGL();
    }

    void LineRenderer::delGL(){
        if(vao != 0)
            glFunc->glDeleteVertexArrays(1, &vao);
        if(vbo != 0)
            glFunc->glDeleteBuffers(1, &vbo);
        vao = 0;
        vbo = 0;
    }

    void LineRenderer::genGL() {
        delGL();
        glFunc->glGenVertexArrays(1, &vao);
        glFunc->glBindVertexArray(vao);
        glFunc->glGenBuffers(1, &vbo);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glFunc->glBindVertexArray(0);
    }

    void LineRenderer::draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        glFunc->glUseProgram(getProgram());
        setMVPUniform(viewMatrix, projectionMatrix);

    }

    GLuint LineRenderer::getProgram() {
        if(mProgram == 0){
            vertexShaderSource = Utils::readGLSLFile(shaderPath + "/lineVertexShaderSource.glsl");
            fragmentShaderSource = Utils::readGLSLFile(shaderPath + "/lineFragmentShaderSource.glsl");
            mProgram = compileProgram();
        }
        return mProgram;
    }

}