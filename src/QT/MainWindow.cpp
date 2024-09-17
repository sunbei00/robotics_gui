#include <QMenuBar>
#include <QBoxLayout>
#include <QScreen>
#include <QGuiApplication>
#include <QToolBar>

#include "QT/MainWindow.h"
#include "QT/GraphicalWidget.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent){
    this->mQNode = std::make_shared<QNode>();
    this->mQNode->start();

    setWindowTitle("Robotics GUI");
    setFullDisplay();

    constructMenubar();
    constructToolbar();
    constructMainWidget();
}

void MainWindow::constructMenubar() {
    auto *menuBar = new QMenuBar(this);

    QMenu *fileMenu = new QMenu("File", this);
    QAction *exitAction = new QAction("Exit", this);
    connect(exitAction, &QAction::triggered, this, &MainWindow::close);
    fileMenu->addAction(exitAction);
    // To Do : load *.pcd
    menuBar->addMenu(fileMenu);

    QMenu *helpMenu = new QMenu("Help", this);
    QAction *aboutAction = new QAction("About", this);
    // connect(aboutAction, &QAction::triggered, this, &MainWindow::showAboutDialog);
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
    connect(viewer, &QAction::triggered, this, [this]() {selectMainWidget(0);});
    connect(flag, &QAction::triggered, this, [this]() {selectMainWidget(1);});
    connect(mapFix, &QAction::triggered, this, [this]() {selectMainWidget(2);});

    this->addToolBar(Qt::LeftToolBarArea, toolBar);
}

void MainWindow::constructMainWidget() {
    mainWidget = new OpenGLWidget(this);
    this->setCentralWidget(mainWidget);
}

void MainWindow::setFullDisplay() {
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->geometry();
    int screenWidth = screenGeometry.width();
    int screenHeight = screenGeometry.height();

    this->resize(screenWidth, screenHeight);
}

void MainWindow::selectMainWidget(int index) {
    // To do : select renderer
}
