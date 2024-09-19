//
// Created by root on 9/12/24.
//

#include "QT/GraphicalWidget.h"
#include "Utils/LoadPCD.h"

#include <glm.hpp>
#include <QMouseEvent>

#include "Graphics/PointRenderer.h"

OpenGLWidget::OpenGLWidget(QWidget *parent)
        : QOpenGLWidget(parent), mCamera(true) {

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &OpenGLWidget::widgetUpdate);
    timer->start(16); // 60 fps
}

OpenGLWidget::~OpenGLWidget() {
    timer->stop();
    delete timer;

    for(auto& it : mRenderer)
        delete it.second;
}

void OpenGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.188235294, 0.188235294, 0.188235294, 1.0f);
}

void OpenGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    mCamera.setWH({(float)w, (float)h});
}

void OpenGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for(auto& it : mRenderer){
        it.second->draw(mCamera.getViewMatrix(), mCamera.getPerspectiveMatrix());
    }
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

