#include "Graphics/OBJLoader.h"
#include "assimp/postprocess.h"

namespace Graphics
{
    void Mesh::setupMesh(QOpenGLFunctions_4_5_Core* glFunc) {
        cleanup(glFunc);

        glFunc->glGenVertexArrays(1, &VAO);
        glFunc->glGenBuffers(1, &VBO);
        glFunc->glGenBuffers(1, &EBO);

        glFunc->glBindVertexArray(VAO);

        glFunc->glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glFunc->glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

        glFunc->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glFunc->glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        glFunc->glEnableVertexAttribArray(0);
        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

        glFunc->glEnableVertexAttribArray(1);
        glFunc->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

        glFunc->glEnableVertexAttribArray(2);
        glFunc->glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));

        glFunc->glBindVertexArray(0);
    }

    void Mesh::cleanup(QOpenGLFunctions_4_5_Core* glFunc) {
        if(VAO !=0)
            glFunc->glDeleteVertexArrays(1, &VAO);
        if(VBO != 0)
            glFunc->glDeleteBuffers(1, &VBO);
        if(EBO != 0)
            glFunc->glDeleteBuffers(1, &EBO);

        VAO=0;
        VBO=0;
        EBO=0;
    }
}


namespace Graphics{

    std::string OBJLoader::meshPath = std::string(MESH_DIR);


    void OBJLoader::loadModel(const std::string& path) {
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            std::cerr << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
            return;
        }

        processNode(scene->mRootNode, scene);
    }

    void OBJLoader::processNode(aiNode* node, const aiScene* scene) {
        for (unsigned int i = 0; i < node->mNumMeshes; i++) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            meshes.push_back(processMesh(mesh, scene));
        }

        for (unsigned int i = 0; i < node->mNumChildren; i++) {
            processNode(node->mChildren[i], scene);
        }
    }

    Mesh OBJLoader::processMesh(aiMesh* mesh, const aiScene* scene) {
        Mesh finalMesh;

        for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
            Vertex vertex;
            vertex.position = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
            if (mesh->HasNormals()) {
                vertex.normal = glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z);
            }
            if (mesh->mTextureCoords[0]) {
                vertex.texCoords = glm::vec2(mesh->mTextureCoords[0][i].x, mesh->mTextureCoords[0][i].y);
            } else {
                vertex.texCoords = glm::vec2(0.0f, 0.0f);
            }
            finalMesh.vertices.push_back(vertex);
        }

        for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
            aiFace face = mesh->mFaces[i];
            for (unsigned int j = 0; j < face.mNumIndices; j++) {
                finalMesh.indices.push_back(face.mIndices[j]);
            }
        }

        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
        finalMesh.material = loadMaterial(material);

        return finalMesh;
    }

    Material OBJLoader::loadMaterial(aiMaterial* material) {
        Material mat;
        aiString name;
        material->Get(AI_MATKEY_NAME, name);
        mat.name = name.C_Str();

        aiColor3D diffuse(0.f, 0.f, 0.f);
        material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
        mat.diffuseColor = glm::vec3(diffuse.r, diffuse.g, diffuse.b);

        aiColor3D specular(0.f, 0.f, 0.f);
        material->Get(AI_MATKEY_COLOR_SPECULAR, specular);
        mat.specularColor = glm::vec3(specular.r, specular.g, specular.b);

        aiColor3D ambient(0.f, 0.f, 0.f);
        material->Get(AI_MATKEY_COLOR_AMBIENT, ambient);
        mat.ambientColor = glm::vec3(ambient.r, ambient.g, ambient.b);

        aiColor3D emissive(0.f, 0.f, 0.f);
        material->Get(AI_MATKEY_COLOR_EMISSIVE, emissive);
        mat.emissiveColor = glm::vec3(emissive.r, emissive.g, emissive.b);

        material->Get(AI_MATKEY_SHININESS, mat.shininess);

        float opacity;
        if (material->Get(AI_MATKEY_OPACITY, opacity) == AI_SUCCESS) {
            mat.dissolve = opacity;
        } else {
            mat.dissolve = 1.0f;
        }

        aiString texturePath;
        if (material->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath) == AI_SUCCESS) {
            mat.diffuseTexturePath = texturePath.C_Str();
        }

        return mat;
    }

    OBJLoader::OBJLoader(const std::string& filePath, QOpenGLFunctions_4_5_Core* glFunc) : mGlFunc(glFunc) {
        loadModel(filePath);
    }

    void OBJLoader::cleanup(){
        for (auto& mesh : meshes) {
            mesh.cleanup(mGlFunc);
        }
    }

    OBJLoader::~OBJLoader() {
        cleanup();
    }

    const std::vector<Mesh>& OBJLoader::getMeshes() const {
      return meshes;
    }

    void OBJLoader::setupMeshes() {
        for (auto& mesh : meshes) {
            mesh.setupMesh(mGlFunc);
        }
    }

}