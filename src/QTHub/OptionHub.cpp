//
// Created by root on 9/21/24.
//

#include "QTHub/OptionHub.h"

namespace QTHub {
    OptionHub* OptionHub::mOptionHub = nullptr;

    OptionHub::OptionHub(QObject* parent) : QObject(parent) {

    }
    OptionHub* OptionHub::getSingleton() {
        if (mOptionHub == nullptr)
            mOptionHub = new OptionHub();
        return mOptionHub;
    }

    void OptionHub::setHubParent(QObject* parent) {
        setParent(parent);
    }

    void OptionHub::setTopView(bool isTopView) {
        emit sTopView(isTopView);
    }

    void OptionHub::setRobotTracking(bool isTracking) {
        emit sRobotTracking(isTracking);
    }

    void OptionHub::clearMap() {
        emit sClearMap();
    }

    void OptionHub::selectOptionMenu(int idx) {
        emit sSelectOptionMenu(idx);
    }

    void OptionHub::undoFlag() {
        emit sUndoFlag();
    }

    void OptionHub::resetFlag() {
        emit sResetFlag();
    }

    void OptionHub::sendFlag() {
        emit sSendFlag();
    }
}

