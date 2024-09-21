#include <QGuiApplication>
#include <QStackedWidget>
#include <QFileDialog>
#include <QDockWidget>
#include <QBoxLayout>
#include <QToolBar>
#include <QMenuBar>
#include <QScreen>

#include "QTHub/OptionHub.h"
#include "QTHub/RobotHub.h"
#include "QT/MainWindow.h"
#include "QT/GraphicWidget.h"
#include "QT/ViewOptionWidget.h"
#include "QT/PathOptionWidget.h"
#include "Utils/LoadPCD.h"
#include "Utils/GetTime.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    this->mQNode = std::make_shared<QNode>(this);
    this->mQNode->start();

    setWindowTitle("Robotics GUI");
    setFullDisplay();

    constructMenubar();
    constructToolbar();
    constructMainWidget();
    constructDockWidget();

    QTHub::GraphicHub::getSingleton()->setHubParent(this);
    QTHub::OptionHub::getSingleton()->setHubParent(this);
    QTHub::RobotHub::getSingleton()->setHubParent(this);

    connect(this, &MainWindow::sSelectOptionMenu, QTHub::OptionHub::getSingleton(), &QTHub::OptionHub::selectOptionMenu);

    connect(this, &MainWindow::sAddPCD, QTHub::GraphicHub::getSingleton(), &QTHub::GraphicHub::addSeparatedPointCloud);


    selectOption(0);
}

MainWindow::~MainWindow() {
    if (mQNode && mQNode->isRunning()){
        mQNode->rosExit = true;
        mQNode->exit();
        mQNode->wait();
    }
}

void MainWindow::constructMenubar() {
    auto *menuBar = new QMenuBar(this);

    QMenu *fileMenu = new QMenu("File", this);

    QAction *load_pcd = new QAction("load pcd file", this);
    connect(load_pcd, &QAction::triggered, this, &MainWindow::loadPCDFile);

    QAction *exitAction = new QAction("Exit", this);
    connect(exitAction, &QAction::triggered, this, &MainWindow::close);
    fileMenu->addAction(load_pcd);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);

    menuBar->addMenu(fileMenu);

    QMenu *helpMenu = new QMenu("Help", this);
    QAction *aboutAction = new QAction("About", this);
    helpMenu->addAction(aboutAction);
    menuBar->addMenu(helpMenu);

    setMenuBar(menuBar);
}

void MainWindow::constructToolbar(){

    QToolBar* toolBar = new QToolBar(this);

    QAction* viewer = new QAction(QIcon(":/eye.png"), "Camera View", toolBar);
    QAction* flag = new QAction(QIcon(":/flag.png"), "Path Selection", toolBar);
    QAction* mapFix = new QAction(QIcon(":/hammer.png"), "Fix Map", toolBar);
    // To Do : Graphical Viewer

    toolBar->addAction(viewer);
    toolBar->addAction(flag);
    toolBar->addAction(mapFix);

    connect(viewer, &QAction::triggered, this, [this]() {selectOption(0);});
    connect(flag, &QAction::triggered, this, [this]() {selectOption(1);});
    connect(mapFix, &QAction::triggered, this, [this]() {selectOption(2);});

    this->addToolBar(Qt::LeftToolBarArea, toolBar);
}

void MainWindow::constructMainWidget() {
    mainWidget = new OpenGLWidget(this);
    mainWidget->setMinimumWidth(300);
    mainWidget->setMinimumHeight(500);
    this->setCentralWidget(mainWidget);
}

void MainWindow::setFullDisplay() {
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int screenWidth = screenGeometry.width();
    int screenHeight = screenGeometry.height();

    this->resize(screenWidth, screenHeight);
}

void MainWindow::selectOption(int index) {
    // To do : select renderer // filtering
    if(!dockWidget->isVisible())
        dockWidget->show();

    QStackedWidget* stackWidget = dynamic_cast<QStackedWidget*>(dockWidget->widget());
    assert(stackWidget != nullptr);
    stackWidget->setCurrentIndex(index);

    IOptionBase* option = dynamic_cast<IOptionBase*>(stackWidget->currentWidget());
    assert(option != nullptr);
    option->selected();

    emit sSelectOptionMenu(index);
}

void MainWindow::loadPCDFile() {
    QString fileName = QFileDialog::getOpenFileName(nullptr, "Select *.pcd file", ".", "*.pcd");
    if (!fileName.isEmpty()) {
        Graphics::pcd_data data;
        Utils::loadPCD(fileName.toStdString(), data);
        DATA::Field field(Utils::getCurrentTimeInSeconds(), DATA::GET_DATA_METHOD::PCD, DATA::DATA_TYPE::POINT_CLOUD, DATA::DATA_STRUCTURE::SEPARATED);

        emit sAddPCD(data, field);
    }
}


void MainWindow::constructDockWidget() {
    dockWidget = new QDockWidget("Option", this);
    dockWidget->setMinimumWidth(200);

    QStackedWidget *stackedWidget = new QStackedWidget(dockWidget);

    QWidget *page1 = new ViewOption(stackedWidget);
    QWidget *page2 = new PathOptionWidget(stackedWidget);

    stackedWidget->addWidget(page1);
    stackedWidget->addWidget(page2);
    dockWidget->setWidget(stackedWidget);
    addDockWidget(Qt::RightDockWidgetArea, dockWidget);
}




