//
// Created by root on 9/12/24.
//

#include <gtc/quaternion.hpp>
#include <QMouseEvent>
#include <chrono>

#include "QTHub/OptionHub.h"
#include "QTHub/RobotHub.h"
#include "QT/GraphicWidget.h"
#include "Utils/LoadPCD.h"
#include "Utils/GetTime.h"
#include "Graphics/PointRenderer.h"
#include "Graphics/TriangleRenderer.h"

OpenGLWidget::OpenGLWidget(QWidget *parent)
        : QOpenGLWidget(parent), mSelectedOptionMenu(0), mRobotRenderer(DATA::Field(),nullptr){
    mTimer = new QTimer(this);
    connect(mTimer, &QTimer::timeout, this, &OpenGLWidget::widgetUpdate);
    mTimer->start(16); // 60 fps

    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sTopView, this, [this](bool isTopView){mCamera.setTopView(isTopView);});
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sRobotTracking, this, [this](bool isRobotTracking){ mIsRobotTracking = isRobotTracking;});
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sSelectOptionMenu, this, [this](int idx){mSelectedOptionMenu = idx;});
    connect(QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sClearMap, this, &OpenGLWidget::clearMap);

    connect(QTHub::RobotHub::getSingleton(), &QTHub::RobotHub::sSetRobotPose, this, &OpenGLWidget::setRobotPose);

    connect(QTHub::GraphicHub::getSingleton(), &QTHub::GraphicHub::sAddSeparatedPointCloud, this, &OpenGLWidget::addSeparatedPointCloudRenderer);
    connect(QTHub::GraphicHub::getSingleton(), &QTHub::GraphicHub::sAddInterleavedPointCloud, this, &OpenGLWidget::addInterleavedPointCloudRenderer);
}

OpenGLWidget::~OpenGLWidget() {
    mTimer->stop();
    delete mTimer;

    makeCurrent();
    for(auto& it : mRenderer)
        delete it.second;

    delete mRobotRenderer.second;
    delete mFlagRenderer.second;
    doneCurrent();
}

void OpenGLWidget::initializeGL() {
    initializeOpenGLFunctions();
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

}

void OpenGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    mCamera.setWH({(float)w, (float)h});
}

void OpenGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    for(auto& it : mRenderer)
        it.second->draw(mCamera);

    mRobotRenderer.second->draw(mCamera);

    for(glm::vec3 pos : mFlagLists) {
        mFlagRenderer.second->modelMatrix = glm::scale(glm::translate(glm::mat4(1.0f), pos), glm::vec3(0.5f, 0.5f, 0.5f));
        mFlagRenderer.second->draw(mCamera);
    }

}

void OpenGLWidget::mousePressEvent(QMouseEvent* event) {
    mCamera.mousePressed(event->button() == Qt::LeftButton, event->button() == Qt::RightButton,
                         { event->pos().x(),  event->pos().y()});

    // To do : Add if Path Mode Condition
    if(mCamera.getIsTopView() && event->button() == Qt::RightButton)
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

void OpenGLWidget::clearMap() {
    makeCurrent();
    for(auto& it : mRenderer)
        delete it.second;
    doneCurrent();
    mRenderer.clear();
}


