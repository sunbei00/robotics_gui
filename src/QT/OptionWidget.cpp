//
// Created by root on 9/18/24.
//
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>


#include "QT/OptionWidget.h"
#include "Graphics/IGraphicalBase.h"

ViewOption::ViewOption(QWidget *parent) : QWidget(parent){

    setLayout(constructZFilter());
}

QLayout *ViewOption::constructZFilter() {
    auto& min = Graphics::ZFilter::mZMin;
    auto& max = Graphics::ZFilter::mZMax;

    auto& minLimit = Graphics::ZFilter::mLimitZMin;
    auto& maxLimit = Graphics::ZFilter::mLimitZMax;

    QVBoxLayout* layout = new QVBoxLayout();

    QSlider *minSlider = new QSlider(Qt::Horizontal, this);
    QSlider *maxSlider = new QSlider(Qt::Horizontal,this);

    minSlider->setRange(minLimit * 100, maxLimit * 100);
    minSlider->setValue(min * 100);

    maxSlider->setRange(minLimit * 100, maxLimit * 100);
    maxSlider->setValue(max * 100);

    minSlider->setFixedHeight(30);
    maxSlider->setFixedHeight(30);

    QLabel *minLabel = new QLabel("Filter-Z-Min");
    QLabel *maxLabel = new QLabel("Filter-Z-Max");

    minLabel->setFixedHeight(30);
    maxLabel->setFixedHeight(30);

    QDoubleSpinBox* minDoubleSpinBox = new QDoubleSpinBox(this);
    minDoubleSpinBox->setRange(minLimit, maxLimit);
    minDoubleSpinBox->setDecimals(2);
    minDoubleSpinBox->setSingleStep(0.1);
    minDoubleSpinBox->setValue(min);

    QDoubleSpinBox* maxDoubleSpinBox = new QDoubleSpinBox(this);
    maxDoubleSpinBox->setRange(minLimit, maxLimit);
    maxDoubleSpinBox->setDecimals(2);
    maxDoubleSpinBox->setSingleStep(0.1);
    maxDoubleSpinBox->setValue(max);

    layout->addWidget(minLabel);
    layout->addWidget(minDoubleSpinBox);
    layout->addWidget(minSlider);

    layout->addWidget(maxLabel);
    layout->addWidget(maxDoubleSpinBox);
    layout->addWidget(maxSlider);

    layout->addStretch();

    connect(minDoubleSpinBox, &QDoubleSpinBox::valueChanged, [=](float value) {
        auto& min = Graphics::ZFilter::mZMin;
        auto& max = Graphics::ZFilter::mZMax;
        min = std::min(value, max);
        minSlider->setValue(min*100);
        minDoubleSpinBox->setValue(min);
    });
    connect(maxDoubleSpinBox, &QDoubleSpinBox::valueChanged, [=](float value) {
        auto& min = Graphics::ZFilter::mZMin;
        auto& max = Graphics::ZFilter::mZMax;
        max = std::max(value, min);
        maxSlider->setValue(max*100);
        maxDoubleSpinBox->setValue(max);
    });

    connect(minSlider, &QSlider::valueChanged, [=](int value) {
        auto& min = Graphics::ZFilter::mZMin;
        auto& max = Graphics::ZFilter::mZMax;
        float real_value = value / 100.0;
        min = std::min(real_value, max);
        minSlider->setValue(min*100.0);
        minDoubleSpinBox->setValue(min);
    });
    connect(maxSlider, &QSlider::valueChanged, [=](int value) {
        auto& min = Graphics::ZFilter::mZMin;
        auto& max = Graphics::ZFilter::mZMax;
        float real_value = value / 100.0;
        max = std::max(real_value, min);
        maxSlider->setValue(max*100.0);
        maxDoubleSpinBox->setValue(max);
    });

    return layout;
}

ViewOption::~ViewOption() = default;
