//
// Created by root on 9/21/24.
//

#include "QT/PathOptionWidget.h"

#include <QVBoxLayout>
#include <QTHub/OptionHub.h>

PathOptionWidget::PathOptionWidget(QWidget* parent) : IOptionBase(parent) {
    auto* optionHub = QTHub::OptionHub::getSingleton();
    connect(this, &PathOptionWidget::sTopView, optionHub, &QTHub::OptionHub::setTopView);


    QVBoxLayout* All = new QVBoxLayout(this);




    All->addStretch();
    setLayout(All);

}

PathOptionWidget::~PathOptionWidget() = default;

void PathOptionWidget::selected() {
    emit sTopView(true);
}


