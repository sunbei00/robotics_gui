//
// Created by root on 9/18/24.
//
#include "Graphics/Camera.h"
#include <gtc/matrix_transform.hpp>

#include <iostream>

namespace Graphics {

    void InteractionCamera::setMouseMode(MOUSE_STATE mouseState)
    {
        this->mMouseState = mouseState;
    }

    void InteractionCamera::setCamera(glm::vec3 eye, glm::vec3 cen, glm::vec3 up)
    {
        this->mCamera.eye = eye;
        this->mCamera.cen = cen;
        this->mCamera.up = up;
    }

    void InteractionCamera::mousePressed(bool left, bool right, glm::vec2 mousePos){
        if(right)
            mMouseState = MOUSE_STATE::RIGHT_DOWN;
        if(left)
            mMouseState = MOUSE_STATE::LEFT_DOWN;
        mMousePos = mousePos;
    }
    void InteractionCamera::mouseReleased(){
        mMouseState = MOUSE_STATE::IDLE;
    }
    void InteractionCamera::mouseMoved(glm::vec2 mousePos){
        if(mMouseState == MOUSE_STATE::IDLE)
            return;
        if(mMouseState == MOUSE_STATE::LEFT_DOWN)
            moveInteraction(mousePos);
        if(mMouseState == MOUSE_STATE::RIGHT_DOWN)
            rotInteraction(mousePos);
        mMousePos = mousePos;
    }
    void InteractionCamera::moveInteraction(glm::vec2 mousePos){
        const float speed = 0.0006f * glm::length(mCamera.eye - mCamera.cen); // 좌클릭 이동 속도 조절
        mCamera.eye += speed * (glm::normalize(glm::cross(mCamera.cen - mCamera.eye, mCamera.up)) * (mMousePos.x - mousePos.x) + glm::normalize(mCamera.up) * (mousePos.y - mMousePos.y));
        mCamera.cen += speed * (glm::normalize(glm::cross(mCamera.cen - mCamera.eye, mCamera.up)) * (mMousePos.x - mousePos.x) + glm::normalize(mCamera.up) * (mousePos.y - mMousePos.y));
    }
    void InteractionCamera::rotInteraction(glm::vec2 mousePos){
        const float rot_speed = (float)0.001 * glm::length(mCamera.eye - mCamera.cen); // 우클릭 회전 속도 조절
        mCamera.cen += rot_speed * (glm::normalize(glm::cross(mCamera.cen - mCamera.eye, mCamera.up)) * (mousePos.x - mMousePos.x) + glm::normalize(mCamera.up) * (mMousePos.y - mousePos.y));
    }
    void InteractionCamera::mouseWheel(float wheel) {
        static float alpha = 0.99f; // wheel 속도 조정

        if (glm::abs(wheel) >= 0.1f) {
            wheel /= glm::abs(wheel);
            glm::vec3 move = (1.f - alpha) * wheel * (mCamera.cen - mCamera.eye);
            mCamera.eye = mCamera.eye + move;
            mCamera.cen = mCamera.cen + move;
        }
    }

    glm::mat4 InteractionCamera::getViewMatrix() {
        return glm::lookAt(mCamera.eye, mCamera.cen, mCamera.up);
    }

    glm::mat4 InteractionCamera::getPerspectiveMatrix() {
        return glm::perspective(glm::radians(60.0f), (float)mCamera.rect.x / (float)mCamera.rect.y, 0.01f, 1000.0f);
    }

    InteractionCamera::InteractionCamera() : mMouseState(MOUSE_STATE::IDLE),
                                            mCamera({ { 0, 10, 30 }, { 0,0,0 }, { 0,1,0 }, {1920, 1080} }){

    }

    void InteractionCamera::setWH(glm::vec2 wh) {
        mCamera.rect = wh;
    }

    void InteractionCamera::move(glm::vec3 movement) {
        mCamera.cen += movement;
        mCamera.eye += movement;
    }
}