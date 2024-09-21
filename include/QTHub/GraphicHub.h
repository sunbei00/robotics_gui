//
// Created by root on 9/21/24.
//

#ifndef GRAPHICHUB_H
#define GRAPHICHUB_H

#include <QObject>
#include "Graphics/PointRenderer.h"
#include "glm.hpp"

namespace DATA{
    enum class GET_DATA_METHOD{ROS, PCD, OBJ, NONE};
    enum class DATA_TYPE{POINT_CLOUD, MESH, ROBOT, NONE};
    enum class DATA_STRUCTURE {INTERLEAVED, SEPARATED, BLENDER, NONE};

    struct Field{
        long long mTime;
        GET_DATA_METHOD mMethod;
        DATA_TYPE mType;
        DATA_STRUCTURE mDataStructure;

        explicit Field(unsigned int time=0, GET_DATA_METHOD method = GET_DATA_METHOD::NONE, DATA_TYPE type = DATA_TYPE::NONE, DATA_STRUCTURE dataStructure = DATA_STRUCTURE::NONE)
            : mTime(time), mMethod(method), mType(type), mDataStructure(dataStructure){}
    };
}


namespace QTHub {

    class GraphicHub final : public QObject{
        Q_OBJECT
    private:
        static GraphicHub* mGraphicHub;
        explicit GraphicHub(QObject *parent = nullptr);
    public:
        static GraphicHub* getSingleton();
        void setHubParent(QObject *parent);

    signals:
        void sAddInterleavedPointCloud(const std::vector<glm::vec3>& pointCloud, DATA::Field field);
        void sAddSeparatedPointCloud(const  Graphics::pcd_data& pointCloud, DATA::Field field);
    public slots:
        void addInterleavedPointCloud(const std::vector<glm::vec3>& pointCloud, DATA::Field field);
        void addSeparatedPointCloud(const Graphics::pcd_data& pointCloud, DATA::Field field);
    };

}


#endif //GRAPHICHUB_H
