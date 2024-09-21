//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_GRAPHICALWIDGET_H
#define ROBOTICS_GUI_GRAPHICALWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QTimer>
#include "Graphics/IGraphicalBase.h"
#include "Graphics/PointRenderer.h"
#include "Graphics/Camera.h"
#include "QTHub/RobotHub.h"
#include "QTHub/GraphicHub.h"

class OpenGLWidget : public QOpenGLWidget, public QOpenGLFunctions_4_5_Core {
Q_OBJECT

protected:
    Graphics::InteractionCamera mCamera;
    RobotPose mRobotPose;
    bool mIsRobotTracking;

    QTimer* mTimer;
    std::vector<std::pair<DATA::Field, Graphics::IGraphicalBase*>> mRenderer;
    std::pair<DATA::Field, Graphics::IGraphicalBase*> mRobotRenderer;
    std::pair<DATA::Field, Graphics::IGraphicalBase*> mFlagRenderer;
    std::vector<glm::vec3> mFlagLists;

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


private slots:
    void widgetUpdate();

public slots:
    void setRobotPose(RobotPose current);
    void setTopView(bool isTopView);
    void setRobotTracking(bool isRobotTracking);

    void addInterleavedPointCloudRenderer(const std::vector<glm::vec3>& pointCloud, DATA::Field field);
    void addSeparatedPointCloudRenderer(const Graphics::pcd_data& pointCloud, DATA::Field field);

    void clearMap();

};


#endif //ROBOTICS_GUI_GRAPHICALWIDGET_H
