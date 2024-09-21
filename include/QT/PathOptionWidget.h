//
// Created by root on 9/21/24.
//

#ifndef PATHOPTIONWIDGET_H
#define PATHOPTIONWIDGET_H
#include "QT/IOptionBaseWidget.h"

class PathOptionWidget : public IOptionBase{
    Q_OBJECT
private:


public:
    explicit PathOptionWidget(QWidget *parent = nullptr);
    ~PathOptionWidget() override;

    void selected() override;

signals:
    void sTopView(bool isTopView);


};


#endif //PATHOPTIONWIDGET_H
