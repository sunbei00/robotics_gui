//
// Created by root on 24. 9. 12.
//


#ifndef ROBOTICS_GUI_QNODE_H
#define ROBOTICS_GUI_QNODE_H
#include <rclcpp/rclcpp.hpp>
#include <QThread>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "Graphics/Camera.h"

class MainWindow;

class QNode : public QThread{
    Q_OBJECT
private:
    std::shared_ptr<rclcpp::Node> node;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;

    MainWindow* mMainWindow;
private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

signals:
    void receivePointCloud(const std::vector<glm::vec3>& point_cloud);
public:
    QNode(QObject* parent = nullptr);
    ~QNode() override = default;
    void run() override;
};



#endif //ROBOTICS_GUI_QNODE_H
