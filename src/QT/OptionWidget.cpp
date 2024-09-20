//
// Created by root on 9/18/24.
//
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>

#include "QT/OptionWidget.h"
#include "QT/MainWindow.h"
#include "Graphics/IGraphicalBase.h"

ViewOption::ViewOption(MainWindow* mainWindow, QWidget *parent) : QWidget(parent), mMainWindow(mainWindow){
    assert(mMainWindow != nullptr);

    QVBoxLayout* All = new QVBoxLayout();

    All->addWidget(constructZFilter());
    All->addWidget(constructRobotTracking());
    All->addWidget(constructTopView());

    All->addStretch();

    setLayout(All);
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
    connect(trackingMode, &QCheckBox::toggled, mMainWindow, &MainWindow::setRobotTrackingMode);
    if(mIsRobotTracking) {
        trackingMode->setCheckState(Qt::CheckState::Checked);
        emit trackingMode->toggled(true);
    }
    else {
        trackingMode->setCheckState(Qt::CheckState::Unchecked);
        emit trackingMode->toggled(false);
    }

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
    connect(topViewMode, &QCheckBox::toggled, mMainWindow, &MainWindow::setTopView);
    if(mIsTopView) {
        topViewMode->setCheckState(Qt::CheckState::Checked);
        emit topViewMode->toggled(true);
    }
    else {
        topViewMode->setCheckState(Qt::CheckState::Unchecked);
        emit topViewMode->toggled(false);
    }


    topViewLayout->addWidget(topViewLabel);
    topViewLayout->addWidget(topViewMode);

    return topViewWidget;
}

ViewOption::~ViewOption() = default;
