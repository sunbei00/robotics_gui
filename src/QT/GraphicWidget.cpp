//
// Created by root on 9/12/24.
//

#include "QT/GraphicWidget.h"
#include "Utils/LoadPCD.h"

#include <glm.hpp>
#include <QMouseEvent>
#include <gtc/quaternion.hpp>

#include "Graphics/PointRenderer.h"
#include "Graphics/TriangleRenderer.h"

OpenGLWidget::OpenGLWidget(QWidget *parent)
        : QOpenGLWidget(parent), mRobotRenderer(DATA::Field(),nullptr){
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &OpenGLWidget::widgetUpdate);
    timer->start(16); // 60 fps
}

OpenGLWidget::~OpenGLWidget() {
    timer->stop();
    delete timer;

    makeCurrent();
    for(auto& it : mRenderer)
        delete it.second;

    delete mRobotRenderer.second;
    doneCurrent();
}

void OpenGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.188235294, 0.188235294, 0.188235294, 1.0f);

    DATA::Field field(0, DATA::GET_DATA_METHOD::OBJ, DATA::DATA_TYPE::ROBOT);
    mRobotRenderer = {field, new Graphics::OBJLoaderTriangleRenderer(Graphics::OBJLoader::meshPath + "/scoutmini.obj" ,this)};
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
}

void OpenGLWidget::mousePressEvent(QMouseEvent* event) {
    mCamera.mousePressed(event->button() == Qt::LeftButton, event->button() == Qt::RightButton,
                         { event->pos().x(),  event->pos().y()});
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

void OpenGLWidget::addRenderer(DATA::Field field, Graphics::IGraphicalBase* renderer) {
    mRenderer.push_back({field, renderer});
}

void OpenGLWidget::moveCamera(glm::vec3 movement) {
        mCamera.move(movement);
}

void OpenGLWidget::moveRobot(Robot current) {
    // robot obj model error
    glm::mat4 modelRotate = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));

    mRobotRenderer.second->modelMatrix = glm::translate(glm::mat4(1.0f), current.position) * glm::mat4_cast(current.orientation) * modelRotate;
}

void OpenGLWidget::setTopView(bool isTopView) {
    mCamera.setTopView(isTopView);
}

