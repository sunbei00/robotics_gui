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


const char* separatePointVertexFilteredShaderSource = R"glsl(
#version 330 core
layout(location = 0) in float x;
layout(location = 1) in float y;
layout(location = 2) in float z;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform float zMin;
uniform float zMax;

void main()
{
    gl_Position = projection * view * model * vec4(x, y, z, 1.0);
    if (y < zMin || y > zMax) { // OpenGL : y, Robotics : z
        gl_PointSize = 0.0;
    }else{
        gl_PointSize = 1.0;
    }
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
    gl_PointSize = 1.0;
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

// PointRendererSeparated ----------------------
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


        // Notice that exchange Y, Z because of the difference between OpenGL coordinate and robotics coordinates.
        // set vbo
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["x"].size() * sizeof(float), mData["x"].data(), GL_STATIC_DRAW);

        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["z"].size() * sizeof(float), mData["z"].data(), GL_STATIC_DRAW);

        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["y"].size() * sizeof(float), mData["y"].data(), GL_STATIC_DRAW);

        // set vao
        glFunc->glEnableVertexAttribArray(0);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        glFunc->glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glFunc->glEnableVertexAttribArray(1);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glFunc->glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glFunc->glEnableVertexAttribArray(2);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glFunc->glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glFunc->glBindVertexArray(0);
    }


    void PointRendererSeparated::draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        glFunc->glUseProgram(getProgram());
        setMVPUniform(viewMatrix, projectionMatrix);

        GLint pointColorLoc = glFunc->glGetUniformLocation(getProgram(), "pointColor");
        glFunc->glUniform3fv(pointColorLoc, 1, &mColor.x);

        glFunc->glBindVertexArray(vao);

        glFunc->glDrawArrays(GL_POINTS, 0, mData["x"].size());

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


// PointRendererSeparatedFiltered -------------------------------
namespace Graphics{

    GLuint PointRendererSeparatedFiltered::mProgram = 0;

    PointRendererSeparatedFiltered::PointRendererSeparatedFiltered(QOpenGLFunctions_4_5_Core *glFunc, const pcd_data& data) : IPointRenderer(glFunc), mData(data){
        genGL();
    }

    PointRendererSeparatedFiltered::PointRendererSeparatedFiltered(QOpenGLFunctions_4_5_Core *glFunc, pcd_data&& data) : IPointRenderer(glFunc), mData(data){
        genGL();
    }

    PointRendererSeparatedFiltered::~PointRendererSeparatedFiltered(){
        delGL();
    }

    void PointRendererSeparatedFiltered::delGL(){
        if(vao != 0)
            glFunc->glDeleteVertexArrays(1, &vao);

        for (auto& it : vbo)
            if(it != 0){
                glFunc->glDeleteBuffers(1, &it);
                it = 0;
            }
        vao = 0;
    }

    void PointRendererSeparatedFiltered::genGL() {
        delGL();
        vbo.resize(3, 0);
        glFunc->glGenVertexArrays(1, &vao);
        glFunc->glBindVertexArray(vao);
        glFunc->glGenBuffers(3, vbo.data());


        // Notice that exchange Y, Z because of the difference between OpenGL coordinate and robotics coordinates.
        // set vbo
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["x"].size() * sizeof(float), mData["x"].data(), GL_STATIC_DRAW);

        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["z"].size() * sizeof(float), mData["z"].data(), GL_STATIC_DRAW);

        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData["y"].size() * sizeof(float), mData["y"].data(), GL_STATIC_DRAW);

        // set vao
        glFunc->glEnableVertexAttribArray(0);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        glFunc->glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glFunc->glEnableVertexAttribArray(1);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glFunc->glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glFunc->glEnableVertexAttribArray(2);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
        glFunc->glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glFunc->glBindVertexArray(0);
    }

    void PointRendererSeparatedFiltered::draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        glFunc->glUseProgram(getProgram());
        setMVPUniform(viewMatrix, projectionMatrix);

        GLint pointColorLoc = glFunc->glGetUniformLocation(getProgram(), "pointColor");
        glFunc->glUniform3fv(pointColorLoc, 1, &mColor.x);

        GLint zMin = glFunc->glGetUniformLocation(getProgram(), "zMin");
        glFunc->glUniform1f(zMin, ZFilter::mZMin);
        GLint zMax = glFunc->glGetUniformLocation(getProgram(), "zMax");
        glFunc->glUniform1f(zMax, ZFilter::mZMax);

        glFunc->glBindVertexArray(vao);

        glFunc->glDrawArrays(GL_POINTS, 0, mData["x"].size());

        glFunc->glBindVertexArray(0);
        glFunc->glUseProgram(0);
    }

    GLuint PointRendererSeparatedFiltered::getProgram() {
        if(mProgram == 0){
            vertexShaderSource = separatePointVertexFilteredShaderSource;
            fragmentShaderSource = pointFragmentShaderSource;
            mProgram = compileProgram();
        }
        return mProgram;
    }
}


// PointRendererInterleaved ----------------------
namespace Graphics{

    GLuint PointRendererInterleaved::mProgram = 0;

    PointRendererInterleaved::PointRendererInterleaved(QOpenGLFunctions_4_5_Core *glFunc, const std::vector<glm::vec3>& data)
    : IPointRenderer(glFunc), mData(data){
        genGL();
    }
    PointRendererInterleaved::PointRendererInterleaved(QOpenGLFunctions_4_5_Core *glFunc, std::vector<glm::vec3>&& data)
    : IPointRenderer(glFunc), mData(data){
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
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData.size() * sizeof(glm::vec3), mData.data(), GL_STATIC_DRAW);

        glFunc->glEnableVertexAttribArray(0);
        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glFunc->glBindVertexArray(0);
    }


    void PointRendererInterleaved::draw(glm::mat4 viewMatrix, glm::mat4 projectionMatrix) {
        glFunc->glUseProgram(getProgram());
        setMVPUniform(viewMatrix, projectionMatrix);

        GLint pointColorLoc = glFunc->glGetUniformLocation(getProgram(), "pointColor");
        glFunc->glUniform3fv(pointColorLoc, 1, &mColor.x);

        glFunc->glBindVertexArray(vao);

        glFunc->glDrawArrays(GL_POINTS, 0, mData.size());

        glFunc->glBindVertexArray(0);
        glFunc->glUseProgram(0);
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