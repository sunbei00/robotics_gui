//
// Created by root on 9/21/24.
//

#ifndef ROBOTHUB_H
#define ROBOTHUB_H
#include <QObject>


#include "glm.hpp"
#include "detail/type_quat.hpp"

struct RobotPose{
    glm::vec3 position{0,0,0};
    glm::quat orientation{1,0,0,0};
};

namespace QTHub {

    class RobotHub final : public QObject{
        Q_OBJECT
    private:
        static RobotHub* mRobotHub;
        explicit RobotHub(QObject *parent = nullptr);
    public:
        static RobotHub* getSingleton();
        void setHubParent(QObject *parent);

    signals:
        void sSetRobotPose(RobotPose robotPose);
        void sSendPath(std::vector<glm::vec3> path);
    public slots:
        void setRobotPose(const RobotPose& current);
        void sendPath(const std::vector<glm::vec3>& path);
    };

}



#endif //ROBOTHUB_H
