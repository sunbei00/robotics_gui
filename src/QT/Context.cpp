//
// Created by root on 9/19/24.
//
#include "QT/Context.h"

SharedContext* SharedContext::singleton = nullptr;

SharedContext::SharedContext() : glFunc(nullptr), sharedContext(nullptr), offscreenSurface(nullptr) {
    initialize();
}

SharedContext::~SharedContext() {
    delete glFunc;
    delete sharedContext;
    delete offscreenSurface;
}

void SharedContext::initialize() {
    offscreenSurface = new QOffscreenSurface();
    offscreenSurface->setFormat(QSurfaceFormat::defaultFormat());
    offscreenSurface->create();

    sharedContext = new QOpenGLContext();
    sharedContext->setFormat(QSurfaceFormat::defaultFormat());
    sharedContext->create();

    sharedContext->makeCurrent(offscreenSurface);

    glFunc = new QOpenGLFunctions_4_5_Core();
    if (!glFunc->initializeOpenGLFunctions())
        qFatal("Could not initialize OpenGL 4.5 Compatibility functions");

    sharedContext->doneCurrent();
}

SharedContext* SharedContext::getSingleton() {
    if(singleton == nullptr)
        singleton = new SharedContext();
    return singleton;
}

QOpenGLFunctions_4_5_Core* SharedContext::getOpenGLFunctions() const {
    return glFunc;
}

QOpenGLContext* SharedContext::getSharedOpenGLContext() {
    return sharedContext;
}