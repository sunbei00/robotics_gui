#include <QGuiApplication>
#include <QStackedWidget>
#include <QFileDialog>
#include <QDockWidget>
#include <QBoxLayout>
#include <QToolBar>
#include <QMenuBar>
#include <QScreen>

#include "QT/MainWindow.h"
#include "QT/GraphicalWidget.h"
#include "Utils/LoadPCD.h"
#include "Graphics/PointRenderer.h"
#include "QT/OptionWidget.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent){
    this->mQNode = std::make_shared<QNode>(this);
    this->mQNode->start();

    setWindowTitle("Robotics GUI");
    setFullDisplay();

    constructMenubar();
    constructToolbar();
    constructMainWidget();
    constructDockWidget();
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

    QAction* viewer = new QAction(QIcon(":/resources/eye.png"), "Camera View", toolBar);
    QAction* flag = new QAction(QIcon(":/resources/flag.png"), "Path Selection", toolBar);
    QAction* mapFix = new QAction(QIcon(":/resources/hammer.png"), "Fix Map", toolBar);
    // To Do : Graphical Viewer

    toolBar->addAction(viewer);
    toolBar->addAction(flag);
    toolBar->addAction(mapFix);

    // To Do : fix selectMainWidget function
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

    ((QStackedWidget*)dockWidget->widget())->setCurrentIndex(index);

}

void MainWindow::loadPCDFile() {
    QString fileName = QFileDialog::getOpenFileName(nullptr, "Select *.pcd file", ".", "*.pcd");
    if (!fileName.isEmpty()) {
        Graphics::pcd_data data;
        Utils::loadPCD(fileName.toStdString(), data);
        DATA::Field field(0, DATA::GET_DATA_METHOD::PCD, DATA::DATA_TYPE::POINT_CLOUD);
        mainWidget->makeCurrent();
        mainWidget->addRenderer(field, new Graphics::PointRendererSeparatedFiltered(mainWidget, std::move(data)));
        mainWidget->doneCurrent();
    }
}

void MainWindow::constructDockWidget() {
    dockWidget = new QDockWidget("Option", this);
    dockWidget->setMinimumWidth(200);

    QStackedWidget *stackedWidget = new QStackedWidget(dockWidget);

    QWidget *page1 = new ViewOption(this, stackedWidget);
    QWidget *page2 = new QWidget(stackedWidget);

    stackedWidget->addWidget(page1);
    stackedWidget->addWidget(page2);
    dockWidget->setWidget(stackedWidget);
    addDockWidget(Qt::RightDockWidgetArea, dockWidget);
}


void MainWindow::setRobotTrackingMode(bool isTracking) {
    mIsRobotTracking = isTracking;
}

void MainWindow::addPointCloudRenderer(const std::vector<glm::vec3>& point_cloud ) {
    assert(mainWidget != nullptr);

    DATA::Field field(0, DATA::GET_DATA_METHOD::ROS, DATA::DATA_TYPE::POINT_CLOUD);
    mainWidget->makeCurrent();
    mainWidget->addRenderer(field, new Graphics::PointRendererInterleavedFiltered(mainWidget, point_cloud));
    mainWidget->doneCurrent();

}

void MainWindow::setRobotPose(Robot current){
    assert(mainWidget != nullptr);
    if(mainWidget == nullptr)
        return;

    static bool isFirst = true;
    if(isFirst){
        connect(this, &MainWindow::robotMovedIncremental, mainWidget, &OpenGLWidget::moveCamera);
        connect(this, &MainWindow::robotMoved, mainWidget, &OpenGLWidget::moveRobot);
        isFirst = false;
    }
    Robot prev = robot;
    glm::vec3 movement = current.position - prev.position;
    if(mIsRobotTracking)
        emit robotMovedIncremental(movement);

    emit robotMoved(current);

    robot = current;
}
