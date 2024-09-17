//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_GRAPHICALWIDGET_H
#define ROBOTICS_GUI_GRAPHICALWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QTimer>
#include "Graphics/PointRenderer.h"
#include "Graphics/Camera.h"

class OpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
Q_OBJECT

private: // tmp test
    Graphics::IPointRenderer* pointRenderer = nullptr;
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
protected:
    Graphics::InteractionCamera mCamera;
    QTimer* timer;
public:
    explicit OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget() override;
public slots:
    void widgetUpdate();

};


#endif //ROBOTICS_GUI_GRAPHICALWIDGET_H
