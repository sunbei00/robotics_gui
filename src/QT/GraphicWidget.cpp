//
// Created by root on 9/12/24.
//

#include <gtc/quaternion.hpp>
#include <QMouseEvent>
#include <chrono>

#include "Utils/CatmullRomSplineInterporation.h"
#include "Utils/LinearInterpolation.h"
#include "Utils/LoadPCD.h"
#include "Utils/GetTime.h"
#include "Graphics/TriangleRenderer.h"
#include "Graphics/PointRenderer.h"
#include "Graphics/LineRenderer.h"

#include "QT/GraphicWidget.h"
#include "QTHub/OptionHub.h"
#include "QTHub/RobotHub.h"

OpenGLWidget::OpenGLWidget(QWidget *parent)
        : QOpenGLWidget(parent), mSelectedOptionMenu(0), mRobotRenderer(DATA::Field(),nullptr){
    mTimer = new QTimer(this);
    connect(mTimer, &QTimer::timeout, this, &OpenGLWidget::widgetUpdate);
    mTimer->start(16); // 60 fps

    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sTopView, this, [this](bool isTopView){mCamera.setTopView(isTopView);});
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sRobotTracking, this, [this](bool isRobotTracking){ mIsRobotTracking = isRobotTracking;});
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sSelectOptionMenu, this, [this](int idx){mSelectedOptionMenu = idx;});
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sClearMap, this, &OpenGLWidget::clearMap);

    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sUndoFlag, this, [this](){if(!mFlagLists.empty()) mFlagLists.pop_back();});
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sSendFlag, this, &OpenGLWidget::sendPath);
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sResetFlag, this, [this](){mFlagLists.clear(); mIsSent=false;});

    connect(QTHub::RobotHub::getSingleton(), &QTHub::RobotHub::sSetRobotPose, this, &OpenGLWidget::setRobotPose);
    connect(this, &OpenGLWidget::sSendPath ,QTHub::RobotHub::getSingleton(), &QTHub::RobotHub::sendPath);

    connect(QTHub::GraphicHub::getSingleton(), &QTHub::GraphicHub::sAddSeparatedPointCloud, this, &OpenGLWidget::addSeparatedPointCloudRenderer);
    connect(QTHub::GraphicHub::getSingleton(), &QTHub::GraphicHub::sAddInterleavedPointCloud, this, &OpenGLWidget::addInterleavedPointCloudRenderer);
}

OpenGLWidget::~OpenGLWidget() {
    mTimer->stop();
    delete mTimer;

    makeCurrent();
    for(auto& it : mRenderer)
        delete it.second;

    delete mTriangleRobotRenderer.second;
    delete mRobotRenderer.second;
    delete mFlagRenderer.second;
    doneCurrent();
}

void OpenGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.188235294, 0.188235294, 0.188235294, 1.0f);

    DATA::Field robotField(Utils::getCurrentTimeInSeconds(), DATA::GET_DATA_METHOD::OBJ, DATA::DATA_TYPE::ROBOT, DATA::DATA_STRUCTURE::BLENDER);
    DATA::Field flagField(Utils::getCurrentTimeInSeconds(), DATA::GET_DATA_METHOD::OBJ, DATA::DATA_TYPE::MESH, DATA::DATA_STRUCTURE::BLENDER);
    mRobotRenderer = {robotField, new Graphics::OBJLoaderTriangleRenderer(Graphics::OBJLoader::meshPath + "/scoutmini.obj" ,this)};
    mFlagRenderer = {flagField, new Graphics::OBJLoaderTriangleRenderer(Graphics::OBJLoader::meshPath + "/RedFlag.obj" ,this)};


    DATA::Field triangleRobotField(Utils::getCurrentTimeInSeconds(), DATA::GET_DATA_METHOD::NONE, DATA::DATA_TYPE::ROBOT, DATA::DATA_STRUCTURE::NONE);
    std::vector<glm::vec3> triangle = {
        glm::vec3(0.8f, 0.0f, 0.2f),   // Vertex A (Tip)
        glm::vec3(0.0f, -0.2f, 0.2f),  // Vertex B (Bottom Left)
        glm::vec3(0.0f, 0.2f, 0.2f)    // Vertex C (Bottom Right)
    };
    mTriangleRobotRenderer = {triangleRobotField, new Graphics::TriangleRenderer(triangle, this)};

}

void OpenGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    mCamera.setWH({(float)w, (float)h});
}

void OpenGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    for(auto& it : mRenderer)
        it.second->draw(mCamera);

    if(mCamera.getDistance() < 20)
        mRobotRenderer.second->draw(mCamera);
    else
        mTriangleRobotRenderer.second->draw(mCamera);

    for(glm::vec3 pos : mFlagLists) {
        mFlagRenderer.second->modelMatrix = glm::scale(glm::translate(glm::mat4(1.0f), pos), glm::vec3(0.5f, 0.5f, 0.5f));
        mFlagRenderer.second->draw(mCamera);
    }

    if(!mFlagLists.empty()) {
        std::vector<glm::vec3> vertices;
        vertices.push_back(mRobotPose.position);
        vertices.push_back(mRobotPose.position);
        for(glm::vec3 FlagPos : mFlagLists)
            vertices.push_back(FlagPos);

        mPath = mIsSent ? mPath : Utils::sampleCatmullRomSpline(vertices, 20);

        Graphics::IGraphicalBase* lineRenderer = new Graphics::LineRenderer(mPath, this);

        lineRenderer->draw(mCamera);
        delete lineRenderer;
    }

}

void OpenGLWidget::mousePressEvent(QMouseEvent* event) {
    mCamera.mousePressed(event->button() == Qt::LeftButton, event->button() == Qt::RightButton,
                         { event->pos().x(),  event->pos().y()});

    // To do : Add if Path Mode Condition
    if(mSelectedOptionMenu == 1 && event->button() == Qt::RightButton)
        mFlagLists.push_back(mCamera.rayCast(glm::vec2(event->pos().x(), event->pos().y())));
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent* event) {
    mCamera.mouseMoved({event->pos().x(), event->pos().y()});
}

void OpenGLWidget::mouseReleaseEvent(QMouseEvent*) {
    mCamera.mouseReleased();

}

void OpenGLWidget::wheelEvent(QWheelEvent* event){
    mCamera.mouseWheel((float)event->angleDelta().y());
}

void OpenGLWidget::widgetUpdate() {
    update();
}

void OpenGLWidget::setRobotPose(RobotPose current) {
    // model axis align
    glm::mat4 modelRotate = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    mRobotRenderer.second->modelMatrix = glm::translate(glm::mat4(1.0f), current.position) * glm::mat4_cast(current.orientation) * modelRotate;

    glm::vec3 eulerAngles = glm::eulerAngles(current.orientation);
    float yaw = eulerAngles.z;
    glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), current.position);
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0.0f, 0.0f, 1.0f));
    mTriangleRobotRenderer.second->modelMatrix = translationMatrix * rotationMatrix;

    RobotPose prev = mRobotPose;
    glm::vec3 movement = current.position - prev.position;

    if(mIsRobotTracking)
        mCamera.move(movement);
    mRobotPose = current;
}

void OpenGLWidget::addInterleavedPointCloudRenderer(const std::vector<glm::vec3>& pointCloud, DATA::Field field) {
    assert(field.mType == DATA::DATA_TYPE::POINT_CLOUD);
    assert(field.mDataStructure == DATA::DATA_STRUCTURE::INTERLEAVED);
    makeCurrent();
    mRenderer.push_back({field, new Graphics::PointRendererInterleavedFiltered(this, pointCloud)});
    doneCurrent();
}

void OpenGLWidget::addSeparatedPointCloudRenderer(const Graphics::pcd_data& pointCloud, DATA::Field field) {
    assert(field.mType == DATA::DATA_TYPE::POINT_CLOUD);
    assert(field.mDataStructure == DATA::DATA_STRUCTURE::SEPARATED);
    makeCurrent();
    mRenderer.push_back({field, new Graphics::PointRendererSeparatedFiltered(this, pointCloud)});
    doneCurrent();
}

void OpenGLWidget::sendPath() {
    mIsSent = true;
    if(mPath.empty())
        return;

    std::vector<glm::vec3> finalPath;

    for(size_t i = 1; i < mPath.size(); i++){
        glm::vec3& srcPose = mPath[i-1];
        glm::vec3& targetPose = mPath[i];

        constexpr float inter_cm = 4;

        auto interpolated = Utils::interpolateBetweenPoints(srcPose, targetPose, inter_cm);
        for(auto inter_pos : interpolated)
            finalPath.push_back(inter_pos);
    }

    emit sSendPath(finalPath);
}

void OpenGLWidget::clearMap() {
    makeCurrent();
    for(auto& it : mRenderer)
        delete it.second;
    doneCurrent();
    mRenderer.clear();
}


