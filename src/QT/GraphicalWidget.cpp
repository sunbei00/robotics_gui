//
// Created by root on 9/12/24.
//

#include "QT/GraphicalWidget.h"
#include "Utils/LoadPCD.h"

#include <glm.hpp>
#include <QMouseEvent>

OpenGLWidget::OpenGLWidget(QWidget *parent)
        : QOpenGLWidget(parent) {

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &OpenGLWidget::widgetUpdate);
    timer->start(16);
}

OpenGLWidget::~OpenGLWidget() {
}

void OpenGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.188235294, 0.188235294, 0.188235294, 1.0f);

    std::map<std::string, std::vector<float>> data;
    Utils::loadPCD("/root/share/code/PathSamplingGUI/dataset/GlobalMap.pcd", data);
    pointRenderer = new Graphics::PointRendererSeparated(this, data);




}

void OpenGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    mCamera.setWH({(float)w, (float)h});
}

void OpenGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pointRenderer->draw(mCamera.getViewMatrix(), mCamera.getPerspectiveMatrix());
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
