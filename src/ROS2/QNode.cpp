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

    // SLAM to OpenGL coordinates (exchange Y-axis ans Z-axis)
    glm::vec3 curr_position(positionMsg.x, positionMsg.z, positionMsg.y);
    glm::quat curr_orientation(orientationMsg.w, orientationMsg.x, orientationMsg.y, orientationMsg.z);

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
    std::vector<glm::vec3> cloud_points;

    const uint32_t point_step = msg->point_step;
    const auto data = msg->data.data();

    int x_offset = -1, y_offset = -1, z_offset = -1;

    for (const auto& field : msg->fields) {
        if (field.name == "x") x_offset = field.offset;
        if (field.name == "y") y_offset = field.offset;
        if (field.name == "z") z_offset = field.offset;
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
        RCLCPP_ERROR(node->get_logger(), "Invalid point cloud data format.");
        return;
    }

    for (uint32_t i = 0; i < msg->width * msg->height; ++i) {
        const uint8_t* point_ptr = data + i * point_step;

        float x = *reinterpret_cast<const float*>(point_ptr + x_offset);
        float y = *reinterpret_cast<const float*>(point_ptr + y_offset);
        float z = *reinterpret_cast<const float*>(point_ptr + z_offset);


        cloud_points.emplace_back(x, y, z);
    }

    assert(mMainWindow != nullptr);
    static bool isFirst = true;
    if(isFirst){
        connect(this, &QNode::receivePointCloud, mMainWindow, &MainWindow::addRenderer2MainWidget);
        isFirst = false;
    }

    emit receivePointCloud(cloud_points);

    //RCLCPP_INFO(node->get_logger(), "Extracted %zu points", cloud_points.size());
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
            "lio_sam/mapping/cloud_registered", qos, std::bind(&QNode::pointcloud_callback, this, std::placeholders::_1));

}

void QNode::run(){
    while(rclcpp::ok()){
        rclcpp::spin_some(node);
        // RCLCPP_INFO(node->get_logger(), "loop");
    }
    rclcpp::shutdown();
}

