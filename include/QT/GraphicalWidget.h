//
// Created by root on 9/12/24.
//

#ifndef ROBOTICS_GUI_GRAPHICALWIDGET_H
#define ROBOTICS_GUI_GRAPHICALWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QTimer>

class OpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
Q_OBJECT

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

public:
    OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget() override;

};


#endif //ROBOTICS_GUI_GRAPHICALWIDGET_H
