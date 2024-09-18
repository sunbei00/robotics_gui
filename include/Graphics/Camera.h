//
// Created by root on 9/18/24.
//

#ifndef ROBOTICS_GUI_CAMERA_H
#define ROBOTICS_GUI_CAMERA_H

#include "glm.hpp"

namespace Graphics{

    enum class MOUSE_STATE {
        LEFT_DOWN,
        RIGHT_DOWN,
        IDLE
    };

    struct Camera {
        glm::vec3 eye;
        glm::vec3 cen;
        glm::vec3 up;
        glm::vec2 rect; // width, height
    };

    class InteractionCamera {
    private:
        void moveInteraction(glm::vec2 mousePos);
        void rotInteraction(glm::vec2 mousePos);
    private:
        MOUSE_STATE mMouseState;
        glm::vec2 mMousePos;
        Camera mCamera;
    public:
        InteractionCamera();
        void setMouseMode(MOUSE_STATE mouseState);
        void setCamera(glm::vec3 eye, glm::vec3 cen, glm::vec3 up);
        void setWH(glm::vec2 wh);
        void mousePressed(bool left, bool right, glm::vec2 mousePos);
        void mouseReleased();
        void mouseMoved(glm::vec2 mousePos);
        void mouseWheel(float wheel);
        void move(glm::vec3 movement);

        glm::mat4 getViewMatrix();
        glm::mat4 getPerspectiveMatrix();
    };
}

#endif //ROBOTICS_GUI_CAMERA_H
