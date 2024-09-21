//
// Created by root on 9/21/24.
//

#ifndef OPTIONHUB_H
#define OPTIONHUB_H
#include <QObject>

namespace QTHub {

    class OptionHub final : public QObject{
        Q_OBJECT
    private:
        static OptionHub* mOptionHub;
        explicit OptionHub(QObject *parent = nullptr);
    public:
        static OptionHub* getSingleton();
        void setHubParent(QObject *parent);

    signals:
        void sTopView(bool isTopView);
        void sRobotTracking(bool isTrackingMode);
        void sClearMap();
        void sSelectOptionMenu(int idx);
        void sUndoFlag();
        void sResetFlag();
        void sSendFlag();
    public slots:
        void setTopView(bool isTopView);
        void setRobotTracking(bool isTracking);
        void clearMap();
        void selectOptionMenu(int idx);
        void undoFlag();
        void resetFlag();
        void sendFlag();
    };

}


#endif //OPTIONHUB_H
