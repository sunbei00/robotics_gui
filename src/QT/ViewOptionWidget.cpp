//
// Created by root on 9/18/24.
//
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>


#include "QTHub/OptionHub.h"
#include "QT/ViewOptionWidget.h"

#include <QFileDialog>

#include "QT/MainWindow.h"
#include "Graphics/IGraphicalBase.h"


ViewOption::ViewOption(QWidget *parent) : IOptionBase(parent){
    QVBoxLayout* All = new QVBoxLayout(this);

    All->addWidget(constructZFilter());
    All->addWidget(constructRobotTracking());
    All->addWidget(constructTopView());
    All->addWidget(constructClearMap());

    All->addStretch();

    setLayout(All);

    auto* optionHub = QTHub::OptionHub::getSingleton();
    connect(this, &ViewOption::sSetTracking, optionHub, &QTHub::OptionHub::setRobotTracking);
    connect(this, &ViewOption::sTopView, optionHub, &QTHub::OptionHub::setTopView);
}

QWidget* ViewOption::constructZFilter() {
    QWidget* zFilterWidget = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout();
    zFilterWidget->setLayout(layout);

    auto& min = Graphics::ZFilter::mZMin;
    auto& max = Graphics::ZFilter::mZMax;

    auto& minLimit = Graphics::ZFilter::mLimitZMin;
    auto& maxLimit = Graphics::ZFilter::mLimitZMax;

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

    return zFilterWidget;
}

QWidget* ViewOption::constructRobotTracking() {
    QWidget* robotTrackingWidget = new QWidget(this);
    QHBoxLayout* robotTrackingLayout = new QHBoxLayout();
    robotTrackingWidget->setLayout(robotTrackingLayout);

    QLabel* trackingLabel = new QLabel("Tracking Mode: ", robotTrackingWidget);
    QCheckBox* trackingMode = new QCheckBox(robotTrackingWidget);
    connect(trackingMode, &QCheckBox::toggled, QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::setRobotTracking);
    connect(trackingMode, &QCheckBox::toggled, this, [this](bool checked){mIsRobotTracking = checked;});
    trackingMode->setCheckState(mIsRobotTracking ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
    emit sSetTracking(mIsRobotTracking);

    robotTrackingLayout->addWidget(trackingLabel);
    robotTrackingLayout->addWidget(trackingMode);

    return robotTrackingWidget;
}

QWidget* ViewOption::constructTopView() {
    QWidget* topViewWidget = new QWidget(this);
    QHBoxLayout* topViewLayout = new QHBoxLayout();
    topViewWidget->setLayout(topViewLayout);

    QLabel* topViewLabel = new QLabel("Top View Mode: ", topViewWidget);
    QCheckBox* topViewMode = new QCheckBox(topViewWidget);
    connect(topViewMode, &QCheckBox::toggled, QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::setTopView);
    connect(topViewMode, &QCheckBox::toggled, this, [this](bool checked) {mIsTopView = checked;});
    topViewMode->setCheckState(mIsTopView ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
    emit sTopView(mIsTopView);

    topViewLayout->addWidget(topViewLabel);
    topViewLayout->addWidget(topViewMode);

    return topViewWidget;
}

QWidget* ViewOption::constructClearMap() {
    QWidget* widget = new QWidget(this);
    QHBoxLayout* layout = new QHBoxLayout();
    widget->setLayout(layout);

    QPushButton* button = new QPushButton(widget);
    button->setText("Clear Map");
    connect(button, &QPushButton::clicked, QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::clearMap);

    layout->addWidget(button);

    return widget;
}

ViewOption::~ViewOption() = default;

void ViewOption::selected() {
    emit sSetTracking(mIsRobotTracking);
    emit sTopView(mIsTopView);
}
