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

namespace DATA{
    enum class GET_DATA_METHOD{ROS, PCD};

    struct Field{
        unsigned int time;
        GET_DATA_METHOD method;
    };
}

class OpenGLWidget : public QOpenGLWidget, public QOpenGLFunctions_4_5_Core
{
Q_OBJECT

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
    std::vector<std::pair<DATA::Field, Graphics::IGraphicalBase*>> mRenderer;
public:
    explicit OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget() override;

    void addRenderer(DATA::Field field, Graphics::IGraphicalBase* renderer);

public slots:
    void widgetUpdate();
    void moveCamera(glm::vec3 movement);

};


#endif //ROBOTICS_GUI_GRAPHICALWIDGET_H
