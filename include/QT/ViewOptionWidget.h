//
// Created by root on 9/18/24.
//

#ifndef ROBOTICS_GUI_OPTIONWIDGET_H
#define ROBOTICS_GUI_OPTIONWIDGET_H

#include "QT/IOptionBaseWidget.h"

class ViewOption final : public IOptionBase{
    Q_OBJECT
private:
    bool mIsTopView = true;
    bool mIsRobotTracking = false;
protected:
    QWidget* constructZFilter();
    QWidget* constructRobotTracking();
    QWidget* constructTopView();
public:
    explicit ViewOption(QWidget* parent = nullptr);
    ~ViewOption() override;

    void selected() override;
signals:
    void sSetTracking(bool isRobotTracking);
    void sTopView(bool isTopView);
};



#endif //ROBOTICS_GUI_OPTIONWIDGET_H
