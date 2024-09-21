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
    public slots:
        void setTopView(bool isTopView);
        void setRobotTracking(bool isTracking);
        void clearMap();
    };

}


#endif //OPTIONHUB_H
