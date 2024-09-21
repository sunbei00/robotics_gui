//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_MAINWINDOW_H
#define ROBOTICS_GUI_MAINWINDOW_H
#include <QMainWindow>
#include "ROS2/QNode.h"
#include "QT/GraphicWidget.h"
#include "QTHub/GraphicHub.h"


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

    void selectOption(int index);

private slots:
    void loadPCDFile();

signals:
    void sAddPCD(const Graphics::pcd_data& pointCloud, DATA::Field field);

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;
};



#endif //ROBOTICS_GUI_MAINWINDOW_H
