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

signals:
    void robotMoved(glm::vec3 movement);

public: // robot info
    Robot robot;
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;
    void cameraTracking(Robot prev, Robot current);
};



#endif //ROBOTICS_GUI_MAINWINDOW_H
