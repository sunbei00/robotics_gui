//
// Created by root on 24. 9. 12.
//


#ifndef ROBOTICS_GUI_QNODE_H
#define ROBOTICS_GUI_QNODE_H
#include <rclcpp/rclcpp.hpp>
#include <QThread>

class QNode : public QThread{
    Q_OBJECT
private:
    std::shared_ptr<rclcpp::Node> node;
public:
    QNode(QObject* parent = nullptr);
    ~QNode() override = default;
    void run() override;
};



#endif //ROBOTICS_GUI_QNODE_H
