#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "QT/MainWindow.h"

int main(int argc, char** argv){
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    QApplication a(argc, argv);
    MainWindow w(nullptr);
    w.show();
    return a.exec();
}



