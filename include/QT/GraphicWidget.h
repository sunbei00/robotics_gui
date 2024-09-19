//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_GRAPHICALWIDGET_H
#define ROBOTICS_GUI_GRAPHICALWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QTimer>
#include "Graphics/IGraphicalBase.h"
#include "Graphics/Camera.h"
#include "Definition/Robot.h"
#include "Definition/Data.h"

class OpenGLWidget : public QOpenGLWidget, public QOpenGLFunctions_4_5_Core
{
Q_OBJECT

protected:
    Graphics::InteractionCamera mCamera;
    QTimer* timer;
    std::vector<std::pair<DATA::Field, Graphics::IGraphicalBase*>> mRenderer;
    std::pair<DATA::Field, Graphics::IGraphicalBase*> mRobotRenderer;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

public:
    explicit OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget() override;

    void addRenderer(DATA::Field field, Graphics::IGraphicalBase* renderer);

public slots:
    void widgetUpdate();
    void moveCamera(glm::vec3 movement);
    void moveRobot(Robot current);

};


#endif //ROBOTICS_GUI_GRAPHICALWIDGET_H
