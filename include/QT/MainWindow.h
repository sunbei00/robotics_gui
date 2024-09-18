//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_MAINWINDOW_H
#define ROBOTICS_GUI_MAINWINDOW_H
#include <QMainWindow>
#include <QDockWidget>
#include <QStackedWidget>

#include "ROS2/QNode.h"
#include "QT/GraphicalWidget.h"
#include "QT/OptionWidget.h"
#include "detail/type_quat.hpp"

struct Robot{
    glm::vec3 position{0,0,0};
    glm::quat orientation{1,0,0,0};
};

class MainWindow : public QMainWindow{
    Q_OBJECT
private: // ros
    std::shared_ptr<QNode> mQNode;
private: // qt
    OpenGLWidget* mainWidget;
    QDockWidget* dockWidget;
private: // method
    void constructMenubar();
    void constructMainWidget();
    void constructToolbar();
    void constructDockWidget();
    void setFullDisplay();

private slots: // slot
    void selectOption(int index);
    void loadPCDFile();
public slots:
    void setRobotTrackingMode(bool isTracking);
signals:
    void robotMoved(glm::vec3 movement);

public: // robot info
    // To do : add mutex?? -> value is set by QNode. QNode is single thread. is it okay?
    // add mutex to Camera??? -> but event driven. is it okay? -> need to check.
    bool mIsRobotTracking = false;
    Robot robot;
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;
    void robotTracking(Robot prev, Robot current);
};



#endif //ROBOTICS_GUI_MAINWINDOW_H
