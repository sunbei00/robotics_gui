//
// Created by root on 9/18/24.
//

#ifndef ROBOTICS_GUI_OPTIONWIDGET_H
#define ROBOTICS_GUI_OPTIONWIDGET_H
#include <QWidget>

class MainWindow;

class ViewOption : public QWidget{
    Q_OBJECT
private:
    MainWindow* mMainWindow;

    bool mIsTopView = false;
    bool mIsRobotTracking = false;
protected:
    QWidget* constructZFilter();
    QWidget* constructRobotTracking();
    QWidget* constructTopView();
public:
    explicit ViewOption(MainWindow* mainWindow, QWidget* parent = nullptr);
    virtual ~ViewOption();
};



#endif //ROBOTICS_GUI_OPTIONWIDGET_H
