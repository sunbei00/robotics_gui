#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "QT/MainWindow.h"

int main(int argc, char** argv){
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    QSurfaceFormat format;
    format.setVersion(4, 5);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication a(argc, argv);
    MainWindow w(nullptr);
    w.show();
    return a.exec();
}



