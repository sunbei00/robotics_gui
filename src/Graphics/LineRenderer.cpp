//
// Created by root on 9/21/24.
//

#include "Graphics/LineRenderer.h"


namespace Graphics{

    GLuint LineRenderer::mProgram = 0;

    LineRenderer::LineRenderer(const std::vector<glm::vec3>& data, QOpenGLFunctions_4_5_Core *glFunc) : IGraphicalBase(glFunc), mData(data){
        genGL();
    }

    LineRenderer::~LineRenderer(){
        delGL();
    }

    void LineRenderer::delGL(){
        if(vao != 0)
            glFunc->glDeleteVertexArrays(1, &vao);
        if(vbo != 0)
            glFunc->glDeleteVertexArrays(1, &vbo);
        if(ebo != 0)
            glFunc->glDeleteVertexArrays(1, &ebo);
        vao = 0;
        vbo = 0;
        ebo = 0;
    }

    void LineRenderer::genGL() {
        delGL();

        std::vector<GLuint> indices;
        for (size_t i = 0; i < mData.size() - 1; ++i) {
            indices.push_back(i);
            indices.push_back(i + 1);
        }

        glFunc->glGenVertexArrays(1, &vao);
        glFunc->glBindVertexArray(vao);

        glFunc->glGenBuffers(1, &vbo);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glFunc->glBufferData(GL_ARRAY_BUFFER, mData.size() * sizeof(glm::vec3), mData.data(), GL_STATIC_DRAW);

        glFunc->glGenBuffers(1, &ebo);
        glFunc->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glFunc->glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);

        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glFunc->glEnableVertexAttribArray(0);

        glFunc->glBindVertexArray(0);
    }

    void LineRenderer::draw(const InteractionCamera& camera) {
        GLuint shaderProgram = getProgram();
        glFunc->glUseProgram(shaderProgram);
        setMVPUniform(camera.getViewMatrix(), camera.getPerspectiveMatrix());

        GLint pointColorLoc = glFunc->glGetUniformLocation(getProgram(), "lineColor");
        GLint lineWidthLocation = glFunc->glGetUniformLocation(shaderProgram, "lineWidth");

        glFunc->glUniform3fv(pointColorLoc, 1, &mColor.x);
        glFunc->glUniform1f(lineWidthLocation, 0.3f);

        glFunc->glBindVertexArray(vao);
        glFunc->glDrawElements(GL_LINES, (mData.size() - 1) * 2, GL_UNSIGNED_INT, 0);

        glFunc->glBindVertexArray(0);
        glFunc->glUseProgram(0);
    }

    GLuint LineRenderer::getProgram() {
        if(mProgram == 0){
            vertexShaderSource = Utils::readGLSLFile(shaderPath + "/lineVertexShaderSource.glsl");
            geometryShaderSource = Utils::readGLSLFile(shaderPath + "/lineGeometryShaderSource.glsl");
            fragmentShaderSource = Utils::readGLSLFile(shaderPath + "/lineFragmentShaderSource.glsl");

            GLuint shaderProgram = glFunc->glCreateProgram();

            GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource.c_str());
            GLuint geometryShader = compileShader(GL_GEOMETRY_SHADER, geometryShaderSource.c_str());
            GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource.c_str());

            glFunc->glAttachShader(shaderProgram, vertexShader);
            glFunc->glAttachShader(shaderProgram, geometryShader);
            glFunc->glAttachShader(shaderProgram, fragmentShader);
            glFunc->glLinkProgram(shaderProgram);

            GLint success;
            glFunc->glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
            if (!success) {
                char infoLog[512];
                glFunc->glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
                std::cerr << "Shader Program Linking Failed\n" << infoLog << std::endl;
            }

            glFunc->glDeleteShader(vertexShader);
            glFunc->glDeleteShader(geometryShader);
            glFunc->glDeleteShader(fragmentShader);

            mProgram = shaderProgram;
        }
        return mProgram;
    }
}