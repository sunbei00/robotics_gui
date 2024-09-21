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


    // right-hand, z : up, coordinates
    class InteractionCamera {
    private:
        void moveInteraction(glm::vec2 mousePos);
        void rotInteraction(glm::vec2 mousePos);
    private:
        MOUSE_STATE mMouseState;
        glm::vec2 mMousePos;
        Camera mCamera;
        Camera mTopCamera;
        bool mIsTopView;

        float mTranslationSpeed = 1.0;
        float mRotationSpeed = 1.0;
    public:
        explicit InteractionCamera(bool isTopView = false);
        void setTopView(bool isTopView);
        void setMouseMode(MOUSE_STATE mouseState);
        void setCamera(glm::vec3 eye, glm::vec3 cen);
        void setWH(glm::vec2 wh);
        void mousePressed(bool left, bool right, glm::vec2 mousePos);
        void mouseReleased();
        void mouseMoved(glm::vec2 mousePos);
        void mouseWheel(float wheel);
        void move(glm::vec3 movement);
        glm::vec3 rayCast(glm::vec2 mousePos);

        bool getIsTopView() const;
        glm::vec3 getEyePos() const;
        glm::vec3 getCenPos() const;
        glm::mat4 getViewMatrix() const;
        glm::mat4 getPerspectiveMatrix() const;
    };
}

#endif //ROBOTICS_GUI_CAMERA_H
