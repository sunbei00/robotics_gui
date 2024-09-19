//
// Created by root on 9/19/24.
//

#ifndef CONTEXT_H
#define CONTEXT_H

#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions_4_5_Core>

class SharedContext final {
private:
    static SharedContext* singleton;
    QOpenGLFunctions_4_5_Core* glFunc;
    QOpenGLContext* sharedContext;
    QOffscreenSurface* offscreenSurface;
private:
    SharedContext();
    ~SharedContext();

    void initialize();
public:
    static SharedContext* getSingleton();
    QOpenGLFunctions_4_5_Core* getOpenGLFunctions() const;
    QOpenGLContext* getSharedOpenGLContext();
};


#endif //CONTEXT_H
