//
// Created by root on 9/13/24.
//

#include "Graphics/LineRenderer.h"

const char* lineVertexShaderSource = R"glsl(
#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)glsl";

const char* lineFragmentShaderSource = R"glsl(
#version 330 core
out vec4 FragColor;

uniform vec3 lineColor;

void main()
{
    FragColor = vec4(lineColor, 1.0);
}
)glsl";

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
            vertexShaderSource = lineVertexShaderSource;
            fragmentShaderSource = lineFragmentShaderSource;
            mProgram = compileProgram();
        }
        return mProgram;
    }

}