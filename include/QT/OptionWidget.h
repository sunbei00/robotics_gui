//
// Created by root on 9/18/24.
//

#ifndef ROBOTICS_GUI_OPTIONWIDGET_H
#define ROBOTICS_GUI_OPTIONWIDGET_H
#include <QWidget>

class ViewOption : public QWidget{
    Q_OBJECT
private:

protected:
    QLayout* constructZFilter();
public:
    explicit ViewOption(QWidget* parent = nullptr);
    virtual ~ViewOption();

};



#endif //ROBOTICS_GUI_OPTIONWIDGET_H
