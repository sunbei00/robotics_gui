//
// Created by root on 9/21/24.
//

#include "QTHub/GraphicHub.h"


namespace QTHub {
    GraphicHub* GraphicHub::mGraphicHub = nullptr;

    GraphicHub::GraphicHub(QObject* parent) : QObject(parent) {

    }
    GraphicHub* GraphicHub::getSingleton() {
        if (mGraphicHub == nullptr)
            mGraphicHub = new GraphicHub();
        return mGraphicHub;
    }

    void GraphicHub::setHubParent(QObject* parent) {
        setParent(parent);
    }

    void GraphicHub::addInterleavedPointCloud(const std::vector<glm::vec3>& pointCloud, DATA::Field field) {
        assert(field.mType == DATA::DATA_TYPE::POINT_CLOUD);
        assert(field.mDataStructure == DATA::DATA_STRUCTURE::INTERLEAVED);

        emit sAddInterleavedPointCloud(pointCloud, field);
    }

    void GraphicHub::addSeparatedPointCloud(const Graphics::pcd_data& pointCloud, DATA::Field field) {
        assert(field.mType == DATA::DATA_TYPE::POINT_CLOUD);
        assert(field.mDataStructure == DATA::DATA_STRUCTURE::SEPARATED);

        emit sAddSeparatedPointCloud(pointCloud, field);
    }
}
