//
// Created by root on 9/18/24.
//
#include "Graphics/Camera.h"
#include <gtc/matrix_transform.hpp>
#include <algorithm>
#include <iostream>

namespace Graphics {

    InteractionCamera::InteractionCamera(bool isTopView) : mMouseState(MOUSE_STATE::IDLE), mIsTopView(false) {
        mCamera = { { 0.0, 10, 8 }, { 0,0,0 }, { 0,0,1 }, {1920, 1080} };
        mTopCamera.up = glm::vec3(0.0f, 1.0f, 0.0f);
        setTopView(isTopView);
    }

    void InteractionCamera::setTopView(bool isTopView)  {
        if (mIsTopView == isTopView)
            return;

        if (isTopView) {
            float distance = glm::length(mCamera.cen - mCamera.eye);
            mTopCamera.cen = mCamera.cen;
            mTopCamera.cen.z = 0;
            mTopCamera.eye = glm::vec3(mCamera.cen.x, mCamera.cen.y, distance);
        } else {
            float distance = glm::length(mTopCamera.cen - mTopCamera.eye);
            glm::vec3 direction = glm::normalize( mCamera.eye - mCamera.cen);
            mCamera.cen = mTopCamera.cen;
            mCamera.cen.z = 0;
            mCamera.eye = mCamera.cen + distance * direction;
        }

        mIsTopView = isTopView;
    }

    void InteractionCamera::setMouseMode(MOUSE_STATE mouseState) {
        this->mMouseState = mouseState;
    }

    void InteractionCamera::setCamera(glm::vec3 eye, glm::vec3 cen) {
        this->mCamera.eye = eye;
        this->mCamera.cen = cen;

        float length = glm::length(mCamera.cen - mCamera.eye);
        mTopCamera.cen = mCamera.cen;
        mTopCamera.eye = glm::vec3(mCamera.cen.x, mCamera.cen.y, length);
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
        if(mMouseState == MOUSE_STATE::RIGHT_DOWN && !mIsTopView)
            rotInteraction(mousePos);
        mMousePos = mousePos;
    }
    void InteractionCamera::moveInteraction(glm::vec2 mousePos){
        const float speed = mTranslationSpeed * 0.0004f * glm::length(mCamera.eye - mCamera.cen); // 좌클릭 이동 속도 조절

        glm::vec3 move = speed * (glm::normalize(glm::cross(mCamera.cen - mCamera.eye, mCamera.up)) * (mMousePos.x - mousePos.x) + glm::normalize(mCamera.up) * (mousePos.y - mMousePos.y));
        mCamera.cen += move;
        mCamera.eye += move;

        move = speed * (glm::normalize(glm::cross(mTopCamera.cen - mTopCamera.eye, mTopCamera.up)) * (mMousePos.x - mousePos.x) + glm::normalize(mTopCamera.up) * (mousePos.y - mMousePos.y));
        mTopCamera.cen += move;
        mTopCamera.eye += move;
    }

    void InteractionCamera::rotInteraction(glm::vec2 mousePos){
        // Math Coordinate - right-hand, z : up

        const float rot_speed = mRotationSpeed * 0.0008f;
        float deltaX = mousePos.x - mMousePos.x;
        float deltaY = mousePos.y - mMousePos.y;

        glm::vec3 dir = glm::normalize(mCamera.cen - mCamera.eye);
        float distance = glm::length(mCamera.cen - mCamera.eye);

        float latitude = std::atan2(dir.y, dir.x);
        float longitude = std::acos(dir.z);

        longitude = longitude + deltaY * rot_speed;;
        latitude = latitude - deltaX * rot_speed;

        const float epsilon = 0.001f;
        longitude = glm::clamp(longitude, epsilon, glm::pi<float>() - epsilon);

        dir.x = std::sin(longitude) * std::cos(latitude);
        dir.y = std::sin(longitude) * std::sin(latitude);
        dir.z = std::cos(longitude);
        dir = glm::normalize(dir);
        mCamera.cen = mCamera.eye + dir * distance;

        glm::vec3 map_up = glm::vec3(0,0,1);

        glm::vec3 right = glm::normalize(glm::cross(dir, map_up));
        mCamera.up = glm::normalize(glm::cross(right, dir));
    }


    void InteractionCamera::mouseWheel(float wheel) {
        static float alpha = 0.95f; // wheel 속도 조정

        if (glm::abs(wheel) >= 0.1f) {
            wheel /= glm::abs(wheel);
            glm::vec3 move = (1.f - alpha) * wheel * (mCamera.cen - mCamera.eye);
            mCamera.eye = mCamera.eye + move;

            move = (1.f - alpha) * wheel * (mTopCamera.cen - mTopCamera.eye);
            mTopCamera.eye = mTopCamera.eye + move;
        }
    }

    bool InteractionCamera::getIsTopView() const {
        return mIsTopView;
    }

    glm::mat4 InteractionCamera::getViewMatrix() const{
        Camera tempCamera = mIsTopView ? mTopCamera : mCamera;

        //glm::vec3 map_up = glm::vec3(0,0,1);
        //return glm::lookAt(tempCamera.eye, tempCamera.cen, (mIsTopView) ? tempCamera.up : map_up);
        return glm::lookAt(tempCamera.eye, tempCamera.cen, tempCamera.up);
    }

    glm::vec3 InteractionCamera::getEyePos() const{
        return (mIsTopView) ? mTopCamera.eye : mCamera.eye;
    }
    glm::vec3 InteractionCamera::getCenPos() const{
        return (mIsTopView) ? mTopCamera.cen : mCamera.cen;
    }

    glm::mat4 InteractionCamera::getPerspectiveMatrix() const{
        Camera tempCamera = mIsTopView ? mTopCamera : mCamera;
        return glm::perspective(glm::radians(60.0f), (float)tempCamera.rect.x / (float)tempCamera.rect.y, 0.01f, 1000.0f);
    }

    void InteractionCamera::setWH(glm::vec2 wh) {
        mCamera.rect = wh;
        mTopCamera.rect = wh;
    }

    void InteractionCamera::move(glm::vec3 movement) {
        mCamera.cen += movement;
        mCamera.eye += movement;

        mTopCamera.cen += movement;
        mTopCamera.eye += movement;
    }
}