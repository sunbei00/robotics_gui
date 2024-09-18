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
protected:
    QLayout* constructZFilter();
    QWidget* constructRobotTracking();
public:
    explicit ViewOption(MainWindow* mainWindow, QWidget* parent = nullptr);
    virtual ~ViewOption();

};



#endif //ROBOTICS_GUI_OPTIONWIDGET_H
