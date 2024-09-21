//
// Created by root on 9/21/24.
//

#include "QTHub/RobotHub.h"

namespace QTHub{
    RobotHub* RobotHub::mRobotHub = nullptr;

    RobotHub::RobotHub(QObject* parent) : QObject(parent) {

    }
    RobotHub* RobotHub::getSingleton() {
        if (mRobotHub == nullptr)
            mRobotHub = new RobotHub();
        return mRobotHub;
    }

    void RobotHub::setHubParent(QObject* parent) {
        setParent(parent);
    }

    void RobotHub::setRobotPose(const RobotPose& current) {
        emit sSetRobotPose(current);
    }

    void RobotHub::sendPath(const std::vector<glm::vec3>& path) {
        emit sSendPath(path);
    }
}
