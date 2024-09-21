//
// Created by root on 9/21/24.
//

#include "QT/PathOptionWidget.h"

#include <QVBoxLayout>

PathOptionWidget::PathOptionWidget(QWidget* parent) : IOptionBase(parent) {


}

PathOptionWidget::~PathOptionWidget() = default;

void PathOptionWidget::selected() {
    QVBoxLayout* All = new QVBoxLayout(this);




    All->addStretch();
    setLayout(All);
}


