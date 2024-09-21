//
// Created by root on 9/21/24.
//

#ifndef OPTIONBASEWIDGET_H
#define OPTIONBASEWIDGET_H

#include <QWidget>

class IOptionBase : public QWidget {
    Q_OBJECT
public:
    explicit IOptionBase(QWidget* parent = nullptr) : QWidget(parent){};
    virtual ~IOptionBase() {};

    virtual void selected() = 0;
};

#endif //OPTIONBASEWIDGET_H
