//
// Created by root on 9/13/24.
//


#include "Graphics/TriangleRenderer.h"

#include <gtc/type_ptr.hpp>


namespace Graphics{

    GLuint OBJLoaderTriangleRenderer::mProgram = 0;

    OBJLoaderTriangleRenderer::OBJLoaderTriangleRenderer(const std::string& objFilePath, QOpenGLFunctions_4_5_Core *glFunc) : IGraphicalBase(glFunc){
        objLoader = new OBJLoader(objFilePath, glFunc);
        genGL();
    }

    OBJLoaderTriangleRenderer::~OBJLoaderTriangleRenderer(){
        if(objLoader != nullptr)
            delete objLoader;
    }

    void OBJLoaderTriangleRenderer::delGL(){
        objLoader->cleanup();
    }

    void OBJLoaderTriangleRenderer::genGL() {
        delGL();
        objLoader->setupMeshes();
    }

    void OBJLoaderTriangleRenderer::draw(const InteractionCamera& camera) {
        GLuint shaderProgram = getProgram();

        glFunc->glUseProgram(shaderProgram);
        setMVPUniform(camera.getViewMatrix(), camera.getPerspectiveMatrix());

        GLint viewPosLoc    = glFunc->glGetUniformLocation(shaderProgram, "viewPos");
        glFunc->glUniform3fv(viewPosLoc, 1, glm::value_ptr(camera.getEyePos()));

        GLint lightPosLoc        = glFunc->glGetUniformLocation(shaderProgram, "light.direction");
        GLint lightAmbientLoc    = glFunc->glGetUniformLocation(shaderProgram, "light.ambient");
        GLint lightDiffuseLoc    = glFunc->glGetUniformLocation(shaderProgram, "light.diffuse");
        GLint lightSpecularLoc   = glFunc->glGetUniformLocation(shaderProgram, "light.specular");
        glFunc->glUniform3fv(lightPosLoc, 1, glm::value_ptr(Light::direction));
        glFunc->glUniform3fv(lightAmbientLoc,  1,glm::value_ptr(Light::ambient));
        glFunc->glUniform3fv(lightDiffuseLoc,  1,glm::value_ptr(Light::diffuse));
        glFunc->glUniform3fv(lightSpecularLoc, 1,glm::value_ptr(Light::specular));

        const auto& meshes = objLoader->getMeshes();
        for (const auto& mesh : meshes) {
            GLint matAmbientLoc     = glFunc->glGetUniformLocation(shaderProgram, "material.ambient");
            GLint matDiffuseLoc     = glFunc->glGetUniformLocation(shaderProgram, "material.diffuseColor");
            GLint matSpecularLoc    = glFunc->glGetUniformLocation(shaderProgram, "material.specular");
            GLint matShininessLoc   = glFunc->glGetUniformLocation(shaderProgram, "material.shininess");

            glFunc->glUniform3fv(matAmbientLoc, 1, glm::value_ptr(mesh.material.ambientColor));
            glFunc->glUniform3fv(matDiffuseLoc, 1, glm::value_ptr(mesh.material.diffuseColor));
            glFunc->glUniform3fv(matSpecularLoc, 1, glm::value_ptr(mesh.material.specularColor));
            glFunc->glUniform1f(matShininessLoc, mesh.material.shininess);

            glFunc->glBindVertexArray(mesh.VAO);
            glDrawElements(GL_TRIANGLES, mesh.indices.size(), GL_UNSIGNED_INT, 0);
            glFunc->glBindVertexArray(0);
        }

    }

    GLuint OBJLoaderTriangleRenderer::getProgram() {
        if(mProgram == 0){
            vertexShaderSource = Utils::readGLSLFile(shaderPath + "/OBJLoaderTraingleVertexShaderCode.glsl");
            fragmentShaderSource = Utils::readGLSLFile(shaderPath + "/OBJLoaderTriangleFragmentShaderSource.glsl");
            mProgram = compileProgram();
        }
        return mProgram;
    }
}
