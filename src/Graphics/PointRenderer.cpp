//
// Created by root on 9/13/24.
//

#include "Graphics/PointRenderer.h"

const char* separatePointVertexShaderSource = R"glsl(
#version 330 core
layout(location = 0) in float x;
layout(location = 1) in float y;
layout(location = 2) in float z;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(x, y, z, 1.0);
    gl_PointSize = 1.0;
}
)glsl";

const char* interleavedPointVertexShaderSource = R"glsl(
#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    gl_PointSize = 10.0;
}
)glsl";

const char* pointFragmentShaderSource = R"glsl(
#version 330 core
out vec4 FragColor;

uniform vec3 pointColor;

void main()
{
    FragColor = vec4(pointColor, 1.0);
}
)glsl";




namespace Graphics{
    IPointRenderer::IPointRenderer(QOpenGLFunctions_4_5_Core *glFunc) : IGraphicalBase(glFunc){}
    IPointRenderer::~IPointRenderer() = default;
}

namespace Graphics{

    GLuint PointRendererSeparated::mProgram = 0;

    PointRendererSeparated::PointRendererSeparated(QOpenGLFunctions_4_5_Core *glFunc, const pcd_data& data) : IPointRenderer(glFunc), mData(data){
        genGL();
    }

    PointRendererSeparated::PointRendererSeparated(QOpenGLFunctions_4_5_Core *glFunc, pcd_data&& data) : IPointRenderer(glFunc), mData(data){
        genGL();
    }

    PointRendererSeparated::~PointRendererSeparated(){
        delGL();
    }

    void PointRendererSeparated::delGL(){
        if(vao != 0)
            glFunc->glDeleteVertexArrays(1, &vao);

        for (auto& it : vbo)
            if(it != 0){
                glFunc->glDeleteBuffers(1, &it);
                it = 0;
            }
        vao = 0;
    }

    void PointRendererSeparated::genGL() {
        delGL();
        vbo.resize(3, 0);
        glFunc->glGenVertexArrays(1, &vao);
        glFunc->glBindVertexArray(vao);
        glFunc->glGenBuffers(3, vbo.data());

        checkOpenGLError("gen error");
        // set vbo
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["x"].size() * sizeof(float), mData["x"].data(), GL_STATIC_DRAW);

        checkOpenGLError("vbo_x error");
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["z"].size() * sizeof(float), mData["z"].data(), GL_STATIC_DRAW);
        checkOpenGLError("vbo_y error");
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["y"].size() * sizeof(float), mData["y"].data(), GL_STATIC_DRAW);
        checkOpenGLError("vbo_z error");
        // set vao
        glFunc->glEnableVertexAttribArray(0);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        glFunc->glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);
        checkOpenGLError("vao_x error");

        glFunc->glEnableVertexAttribArray(1);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glFunc->glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);
        checkOpenGLError("vbo_y error");

        glFunc->glEnableVertexAttribArray(2);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glFunc->glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);
        checkOpenGLError("vbo_z error");

        glFunc->glBindVertexArray(0);
    }


    void PointRendererSeparated::draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        glFunc->glUseProgram(getProgram());
        setMVPUniform(viewMatrix, projectionMatrix);
        checkOpenGLError("MVP error");

        GLint pointColorLoc = glFunc->glGetUniformLocation(getProgram(), "pointColor");
        glFunc->glUniform3fv(pointColorLoc, 1, &mColor.x);
        checkOpenGLError("pointColor error");

        glFunc->glBindVertexArray(vao);

        glFunc->glDrawArrays(GL_POINTS, 0, mData["x"].size());
        checkOpenGLError("draw error");

        glFunc->glBindVertexArray(0);
        glFunc->glUseProgram(0);
    }

    GLuint PointRendererSeparated::getProgram() {
        if(mProgram == 0){
            vertexShaderSource = separatePointVertexShaderSource;
            fragmentShaderSource = pointFragmentShaderSource;
            mProgram = compileProgram();
        }
        return mProgram;
    }

}


namespace Graphics{

    GLuint PointRendererInterleaved::mProgram = 0;

    PointRendererInterleaved::PointRendererInterleaved(QOpenGLFunctions_4_5_Core *glFunc) : IPointRenderer(glFunc){
        genGL();
    }

    PointRendererInterleaved::~PointRendererInterleaved(){
        delGL();
    }

    void PointRendererInterleaved::delGL(){
        if(vao != 0)
            glFunc->glDeleteVertexArrays(1, &vao);
        if(vbo != 0)
            glFunc->glDeleteBuffers(1, &vbo);
        vao = 0;
        vbo = 0;
    }

    void PointRendererInterleaved::genGL() {
        delGL();
        glFunc->glGenVertexArrays(1, &vao);
        glFunc->glBindVertexArray(vao);
        glFunc->glGenBuffers(1, &vbo);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glFunc->glBindVertexArray(0);
    }


    void PointRendererInterleaved::draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        glFunc->glUseProgram(getProgram());
        setMVPUniform(viewMatrix, projectionMatrix);
    }

    GLuint PointRendererInterleaved::getProgram() {
        if(mProgram == 0){
            vertexShaderSource = interleavedPointVertexShaderSource;
            fragmentShaderSource = pointFragmentShaderSource;
            mProgram = compileProgram();
        }
        return mProgram;
    }
}