//
// Created by root on 9/13/24.
//


#include "Graphics/TriangleRenderer.h"

const char* triangleVertexShaderSource = R"glsl(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;

out vec3 ourColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = aColor;
}
)glsl";

const char* triangleFragmentShaderSource = R"glsl(
#version 330 core
out vec4 FragColor;

in vec3 ourColor;

void main()
{
    FragColor = vec4(ourColor, 1.0);
}
)glsl";

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
            vertexShaderSource = triangleVertexShaderSource;
            fragmentShaderSource = triangleFragmentShaderSource;
            mProgram = compileProgram();
        }
        return mProgram;
    }
}
