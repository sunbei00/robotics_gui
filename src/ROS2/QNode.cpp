//
// Created by root on 24. 9. 12.
//

#include "ROS2/QNode.h"
#include "QT/MainWindow.h"

void QNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    assert(mMainWindow != nullptr);
    if (mMainWindow == nullptr)
        return;
    auto& positionMsg = msg->pose.pose.position;
    auto& orientationMsg = msg->pose.pose.orientation;

    // SLAM to OpenGL coordinates
    glm::vec3 curr_position(positionMsg.x, positionMsg.z, positionMsg.y);
    glm::quat curr_orientation(orientationMsg.w, orientationMsg.x, orientationMsg.z, orientationMsg.y);

    Robot prev_robot = mMainWindow->robot;
    mMainWindow->robot.position = curr_position;
    mMainWindow->robot.orientation = curr_orientation;

    static bool isFirst = true;
    if(isFirst){
        prev_robot = mMainWindow->robot;
        isFirst = false;
    }
    mMainWindow->robotTracking(prev_robot, mMainWindow->robot);
}

void QNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Received point cloud data, width: %u, height: %u",
                msg->width, msg->height);
}

QNode::QNode(QObject* parent) : QThread(parent){
    if(parent != nullptr){
        mMainWindow = dynamic_cast<MainWindow*>(parent);
        assert(mMainWindow != nullptr);
    }
    else
        mMainWindow = nullptr;

    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("robotics_gui");

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    odom_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry", qos, std::bind(&QNode::odom_callback, this, std::placeholders::_1));

    pointcloud_subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", qos, std::bind(&QNode::pointcloud_callback, this, std::placeholders::_1));

}

void QNode::run(){
    while(rclcpp::ok()){
        rclcpp::spin_some(node);
        // RCLCPP_INFO(node->get_logger(), "loop");
    }
    rclcpp::shutdown();
}

