//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_MAINWINDOW_H
#define ROBOTICS_GUI_MAINWINDOW_H
#include <QMainWindow>
#include <QStackedWidget>
#include "ROS2/QNode.h"

class MainWindow : public QMainWindow{
    Q_OBJECT
private: // ros
    std::shared_ptr<QNode> mQNode;
private: // qt
    QStackedWidget* mStackedWidget;
private: // method
    void constructMenubar();
    void constructMainWidget();
    void setFullDisplay();
    void constructToolbar();

private slots: // slot
    void selectMainWidget(int index);

public:
    explicit MainWindow(QWidget* parent = nullptr);
    virtual ~MainWindow() = default;
};



#endif //ROBOTICS_GUI_MAINWINDOW_H
