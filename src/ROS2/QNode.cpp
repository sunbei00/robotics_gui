//
// Created by root on 24. 9. 12.
//

#include "ROS2/QNode.h"
#include "QT/MainWindow.h"
#include "QTHub/GraphicHub.h"
#include "Utils/GetTime.h"


QNode::QNode(QObject* parent) : QThread(parent){
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("robotics_gui");

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    odom_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/imu", qos, std::bind(&QNode::odom_callback, this, std::placeholders::_1));

    pointcloud_subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lio_sam/mapping/cloud_registered", qos, std::bind(&QNode::pointcloud_callback, this, std::placeholders::_1));

    path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("/Planning/local_path", 10);

    connect(this, &QNode::sAddPointCloud, QTHub::GraphicHub::getSingleton(), &QTHub::GraphicHub::addInterleavedPointCloud);
    connect(this, &QNode::sSetRobotPose, QTHub::RobotHub::getSingleton(), &QTHub::RobotHub::setRobotPose);
    connect(QTHub::RobotHub::getSingleton(), &QTHub::RobotHub::sSendPath, this, &QNode::sendTopic);
}

void QNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto& positionMsg = msg->pose.pose.position;
    auto& orientationMsg = msg->pose.pose.orientation;

    RobotPose curr_robot;
    curr_robot.position = glm::vec3(positionMsg.x, positionMsg.y, positionMsg.z);
    curr_robot.orientation = glm::quat(orientationMsg.w, orientationMsg.x, orientationMsg.y, orientationMsg.z);

    emit sSetRobotPose(curr_robot);
}

void QNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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

    DATA::Field field(Utils::getCurrentTimeInSeconds(), DATA::GET_DATA_METHOD::ROS, DATA::DATA_TYPE::POINT_CLOUD, DATA::DATA_STRUCTURE::INTERLEAVED);
    emit sAddPointCloud(cloud_points, field);

    //RCLCPP_INFO(node->get_logger(), "Extracted %zu points", cloud_points.size());
}

void QNode::run(){
    while(rclcpp::ok()){
        if(rosExit)
            break;
        rclcpp::spin_some(node);
        // RCLCPP_INFO(node->get_logger(), "loop");
    }
    rclcpp::shutdown();
}

void QNode::sendTopic(std::vector<glm::vec3> path) {

    rclcpp::Time now = node->now();

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = "map";
    for (const auto &pos : path)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = "map";
        pose.pose.position.x = pos.x;
        pose.pose.position.y = pos.y;
        pose.pose.position.z = pos.z;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    path_publisher_->publish(path_msg);
}

