    //
    // Created by root on 9/21/24.
    //


    #include <QVBoxLayout>
    #include <QPushButton>
    #include <QShortcut>

    #include "QT/PathOptionWidget.h"
    #include "QTHub/OptionHub.h"



    PathOptionWidget::PathOptionWidget(QWidget* parent) : IOptionBase(parent) {
        auto* optionHub = QTHub::OptionHub::getSingleton();
        connect(this, &PathOptionWidget::sTopView, optionHub, &QTHub::OptionHub::setTopView);


        QVBoxLayout* All = new QVBoxLayout(this);

        All->addWidget(constructUndoWidget());
        All->addWidget(constructSendWidget());
        All->addWidget(constructResetWidget());


        All->addStretch();
        setLayout(All);

        emit sButtonAvalidable(mSendCode);
    }

    QWidget* PathOptionWidget::constructUndoWidget() {
        QWidget* widget = new QWidget(this);
        QHBoxLayout* layout = new QHBoxLayout();
        widget->setLayout(layout);

        QPushButton* undoButton = new QPushButton(widget);
        undoButton->setText("Undo");
        connect(undoButton, &QPushButton::clicked, QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::undoFlag);

        layout->addWidget(undoButton);

        QShortcut* shortcut = new QShortcut(QKeySequence("Ctrl+Z"), widget);
        connect(shortcut, &QShortcut::activated, undoButton, &QPushButton::click);

        return widget;
    }
    QWidget* PathOptionWidget::constructSendWidget() {
        QWidget* widget = new QWidget(this);
        QHBoxLayout* layout = new QHBoxLayout();
        widget->setLayout(layout);

        QPushButton* sendButton = new QPushButton(widget);
        sendButton->setText("Send");
        connect(sendButton, &QPushButton::clicked, QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::sendFlag);
        connect(sendButton, &QPushButton::clicked, this, [this](){sendCode(SEND_CODE::SEND);});
        connect(this, &PathOptionWidget::sButtonAvalidable, sendButton, [=](SEND_CODE sendCode){ if(sendCode==SEND_CODE::SEND) sendButton->setEnabled(false); else sendButton->setEnabled(true);} );

        layout->addWidget(sendButton);

        return widget;
    }

    QWidget* PathOptionWidget::constructResetWidget() {
        QWidget* widget = new QWidget(this);
        QHBoxLayout* layout = new QHBoxLayout();
        widget->setLayout(layout);

        QPushButton* resetButton = new QPushButton(widget);
        resetButton->setText("Reset");
        connect(resetButton, &QPushButton::clicked, QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::resetFlag);
        connect(resetButton, &QPushButton::clicked, this, [this](){sendCode(SEND_CODE::RESET);});
        connect(this, &PathOptionWidget::sButtonAvalidable, resetButton, [=](SEND_CODE sendCode){ if(sendCode==SEND_CODE::RESET) resetButton->setEnabled(false); else resetButton->setEnabled(true);} );

        layout->addWidget(resetButton);

        return widget;
    }

    PathOptionWidget::~PathOptionWidget() = default;

    void PathOptionWidget::selected() {
        emit sTopView(true);
    }

    void PathOptionWidget::sendCode(SEND_CODE code) {
        assert(mSendCode != code);

        mSendCode = code;

        emit sButtonAvalidable(mSendCode);
    }



