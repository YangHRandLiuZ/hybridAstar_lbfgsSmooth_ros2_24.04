/**
 * @file visualize.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "rclcpp/rclcpp.hpp"  // 替换 ros/ros.h
#include "tf2/LinearMath/Quaternion.h"  // 替换 tf/transform_datatypes.h（TF2 四元数）
#include "tf2_ros/transform_broadcaster.h"  // TF2 转换工具
#include "geometry_msgs/msg/pose_stamped.hpp"  // ROS 2 消息类型（带 msg:: 命名空间）
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/convert.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // 提供 tf2 与 geometry_msgs 的转换实现
#include "type_defs.h"  // 保留自定义类型

namespace planning
{

class Visualize
{
private:
    // 1. 替换 ROS 1 NodeHandle 为 ROS 2 节点指针
    rclcpp::Node::SharedPtr node_;

    // 2. 替换 ros::Publisher 为 ROS 2 发布者类型（带 msg:: 消息类型）
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr explored_nodes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_boxes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_points_pub_;

    // 3. 消息类型添加 msg:: 命名空间
    geometry_msgs::msg::PoseArray nodes_pose_;
    visualization_msgs::msg::MarkerArray vehicle_boxes_;
    visualization_msgs::msg::MarkerArray path_points_;

public:
    // 4. 构造函数接收 ROS 2 节点指针（外部传入，符合 ROS 2 节点管理逻辑）
    Visualize(rclcpp::Node::SharedPtr node) : node_(node) {
        // 5. 替换 nh.advertise 为 node->create_publisher
        explored_nodes_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
            "/visualize_nodes_pose", 10);
        vehicle_boxes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/vehicle_boxes", 10);
        path_points_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/path_points", 10);
    }

    void clear() {
        nodes_pose_.poses.clear();
        vehicle_boxes_.markers.clear();
        path_points_.markers.clear();  // 补充清理路径点，避免残留
    }

    void publishExploredNodes(Vec3d node_pose) {
        nodes_pose_.header.frame_id = "map";
        // 6. 替换 ros::Time::now() 为节点时钟
        nodes_pose_.header.stamp = node_->get_clock()->now();

        geometry_msgs::msg::Pose pose;  // 消息类型带 msg::
        pose.position.x = node_pose(0);
        pose.position.y = node_pose(1);

        // 7. 替换 tf::createQuaternionMsgFromYaw 为 TF2 方法
        tf2::Quaternion q;
        q.setRPY(0, 0, node_pose(2));  // 滚转、俯仰、偏航（仅偏航有值）
        q.normalize();  // 归一化四元数
        pose.orientation.y = q.y();
        pose.orientation.x = q.x();
        pose.orientation.w = q.w();
        pose.orientation.z = q.z();

        nodes_pose_.poses.push_back(pose);
        explored_nodes_pub_->publish(nodes_pose_);  // 发布者调用 publish()
    }
    
    void publishVehicleBoxes(Vec3d node_pose, int i) {
        visualization_msgs::msg::Marker vehicle_box;  // 消息类型带 msg::

        if (i == 0) {
            vehicle_box.action = visualization_msgs::msg::Marker::DELETEALL;  // 替换 3（ROS 1 常量）为 ROS 2 枚举
        }

        vehicle_box.header.frame_id = "map";
        vehicle_box.header.stamp = rclcpp::Time(0);  // 替换 ros::Time(0)
        vehicle_box.id = i;
        vehicle_box.type = visualization_msgs::msg::Marker::CUBE;
        vehicle_box.scale.x = 4.933;
        vehicle_box.scale.y = 2.11;
        vehicle_box.scale.z = 1;
        vehicle_box.color.a = 0.05;
        vehicle_box.color.r = 0;
        vehicle_box.color.b = 1;
        vehicle_box.color.g = 0;

        Vec2d center = {
            node_pose(0) + 1.45 * std::cos(node_pose(2)),
            node_pose(1) + 1.45 * std::sin(node_pose(2))
        };
        vehicle_box.pose.position.x = center(0);
        vehicle_box.pose.position.y = center(1);

        // 8. 同样使用 TF2 生成偏航角四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, node_pose(2));
        q.normalize();
vehicle_box.pose.orientation.x = q.x();
vehicle_box.pose.orientation.y = q.y();
vehicle_box.pose.orientation.z = q.z();
vehicle_box.pose.orientation.w = q.w();

        vehicle_boxes_.markers.push_back(vehicle_box);
        vehicle_boxes_pub_->publish(vehicle_boxes_);
    }

    void publishPathPoint(Vec2d position, int i) {
        visualization_msgs::msg::Marker path_point;  // 消息类型带 msg::

        if (i == 0) {
            path_point.action = visualization_msgs::msg::Marker::DELETEALL;  // 替换 3 为枚举
        }

        path_point.header.frame_id = "map";
        path_point.header.stamp = rclcpp::Time(0);  // 替换 ros::Time(0)
        path_point.id = i;
        path_point.type = visualization_msgs::msg::Marker::SPHERE;
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.scale.z = 0.1;

        path_point.color.a = 1;
        path_point.color.r = 1;
        path_point.color.g = 0;
        path_point.color.b = 0;

        path_point.pose.position.x = position(0);
        path_point.pose.position.y = position(1);
        path_point.pose.position.z = 0.1;
        path_point.pose.orientation.w = 1.0;

        path_points_.markers.push_back(path_point);
        path_points_pub_->publish(path_points_);
    }
};

} // namespace planning