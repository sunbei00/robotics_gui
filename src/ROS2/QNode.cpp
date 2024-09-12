//
// Created by root on 24. 9. 12.
//

#include "ROS2/QNode.h"


QNode::QNode(QObject* parent) : QThread(parent){
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("robotics_gui");
    // RCLCPP_INFO_ONCE(node->get_logger(), "ros2 node On");
}

void QNode::run(){
    while(rclcpp::ok()){
        rclcpp::spin_some(node);
        // RCLCPP_INFO(node->get_logger(), "loop");
    }
    rclcpp::shutdown();
}

