//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_MAINWINDOW_H
#define ROBOTICS_GUI_MAINWINDOW_H
#include <QMainWindow>
#include "ROS2/QNode.h"
#include "QT/GraphicWidget.h"


class MainWindow : public QMainWindow{
    Q_OBJECT
private: // ros
    std::shared_ptr<QNode> mQNode;
private: // qt
    OpenGLWidget* mainWidget;
    QDockWidget* dockWidget;
private:
    Robot robot;
private: // method
    void constructMenubar();
    void constructMainWidget();
    void constructToolbar();
    void constructDockWidget();
    void setFullDisplay();

private slots:
    void selectOption(int index);
    void loadPCDFile();
public slots:
    void setRobotTrackingMode(bool isTracking);
    void addPointCloudRenderer(const std::vector<glm::vec3>& point_cloud);
    void setRobotPose(Robot current);
signals:
    void robotMovedIncremental(glm::vec3 movement);
    void robotMoved(Robot pose);

public: // robot info
    // To do : add mutex?? -> value is set by QNode. QNode is single thread. is it okay?
    // add mutex to Camera??? -> but event driven. is it okay? -> need to check.
    bool mIsRobotTracking = false;
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;
};



#endif //ROBOTICS_GUI_MAINWINDOW_H
