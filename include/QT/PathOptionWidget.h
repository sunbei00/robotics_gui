//
// Created by root on 9/21/24.
//

#ifndef PATHOPTIONWIDGET_H
#define PATHOPTIONWIDGET_H
#include "QT/IOptionBaseWidget.h"

enum class SEND_CODE {SEND, RESET};

class PathOptionWidget : public IOptionBase{
    Q_OBJECT
private:
    SEND_CODE mSendCode = SEND_CODE::RESET;

    QWidget* constructUndoWidget();
    QWidget* constructSendWidget();
    QWidget* constructResetWidget();


public:
    explicit PathOptionWidget(QWidget *parent = nullptr);
    ~PathOptionWidget() override;

    void selected() override;

signals:
    void sTopView(bool isTopView);
    void sButtonAvalidable(SEND_CODE sendCode);
private slots:
    void sendCode(SEND_CODE code);
};


#endif //PATHOPTIONWIDGET_H
