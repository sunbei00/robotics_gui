//
// Created by root on 9/19/24.
//

#ifndef LOADMODEL_H
#define LOADMODEL_H
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <QOpenGLFunctions_4_5_Core>
#include <glm.hpp>
#include <iostream>
#include <vector>
#include <string>

namespace Graphics{

    struct Vertex {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 texCoords;
    };

    struct Material {
        std::string name;
        std::string diffuseTexturePath;
        glm::vec3 diffuseColor;
        glm::vec3 specularColor;
        glm::vec3 ambientColor;
        glm::vec3 emissiveColor;
        float shininess;
        float dissolve;
    };

    struct Mesh {
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        Material material;
        GLuint VAO=0, VBO=0, EBO=0;

        void setupMesh(QOpenGLFunctions_4_5_Core* glFunc);
        void cleanup(QOpenGLFunctions_4_5_Core* glFunc);
    };

    class OBJLoader {
    private:
        std::vector<Mesh> meshes;
        QOpenGLFunctions_4_5_Core* mGlFunc;
    private:
        void loadModel(const std::string& path);
        void processNode(aiNode* node, const aiScene* scene);
        Mesh processMesh(aiMesh* mesh, const aiScene* scene);
        Material loadMaterial(aiMaterial* material);
    public:
        static std::string meshPath;
    public:
        OBJLoader(const std::string& objFilePath, QOpenGLFunctions_4_5_Core* glFunc);
        ~OBJLoader();
        const std::vector<Mesh>& getMeshes() const;
        void setupMeshes();
        void cleanup();

    };

}
#endif //LOADMODEL_H
