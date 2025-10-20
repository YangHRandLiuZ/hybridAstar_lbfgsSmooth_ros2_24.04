#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/planners/rrt/RRT.h"  
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "tf2/LinearMath/Quaternion.h"
#include <string>
#include <mutex>

// 全局变量（仅用基础类型和double）
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_vis_pub;
grid_map::GridMap map_;
std::mutex map_mutex;

// 地图回调（增加地图参数日志，便于排查类型问题）
void Gridmap_Callback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex);
    // 显式转换消息到grid_map，确保数据类型匹配
    grid_map::GridMapRosConverter::fromMessage(*msg, map_);
    
    // 打印地图关键参数（验证分辨率为double类型，避免隐式类型问题）
    RCLCPP_INFO(rclcpp::get_logger("ompl_test_node"), 
        "Received grid map: resolution=%.3f, size_x=%d, size_y=%d",
        map_.getResolution(),  // double类型
        map_.getSize()(0),     // int类型（栅格数量X轴）
        map_.getSize()(1));    // int类型（栅格数量Y轴）
}

// 状态有效性检查（核心优化：显式转换坐标→栅格索引，避免Eigen类型混用）
bool isStateValid(const ompl::base::State* state) {
    std::lock_guard<std::mutex> lock(map_mutex);
    // 1. 先检查地图是否加载且包含"elevation"层
    if (!map_.exists("elevation")) {
        RCLCPP_DEBUG(rclcpp::get_logger("ompl_test_node"), "Map missing 'elevation' layer");
        return false;
    }

    // 2. 提取当前状态的坐标（double类型，符合路径规划精度需求）
    const auto* dubins_state = state->as<ompl::base::DubinsStateSpace::StateType>();
    const double x = dubins_state->getX();
    const double y = dubins_state->getY();
    const grid_map::Position pos(x, y);  // Eigen::Vector2d（double类型）

    // 3. 显式检查坐标是否在地图边界内（避免越界访问触发类型转换）
    if (!map_.isInside(pos)) {
        RCLCPP_DEBUG(rclcpp::get_logger("ompl_test_node"), 
            "State (%.2f, %.2f) outside map bounds", x, y);
        return false;
    }

    // 4. 显式转换坐标→栅格索引（int类型，核心优化点：避免隐式赋值）
    grid_map::Index grid_index;  // Eigen::Vector2i（int类型）
    if (!map_.getIndex(pos, grid_index)) {  // 显式转换，失败则返回无效
        RCLCPP_DEBUG(rclcpp::get_logger("ompl_test_node"), 
            "Failed to convert (%.2f, %.2f) to grid index", x, y);
        return false;
    }

    // 5. 显式检查栅格索引有效性（避免无效索引访问）
    if (!map_.isValid(grid_index)) {
        RCLCPP_DEBUG(rclcpp::get_logger("ompl_test_node"), 
            "Grid index (%d, %d) is invalid", grid_index(0), grid_index(1));
        return false;
    }

    // 6. 访问地图数据（确保double类型比较，无类型冲突）
    const double elevation = map_.at("elevation", grid_index);
    return elevation < 0.5;  // double vs double，无类型问题
}

int main(int argc, char **argv) {
    // 1. ROS初始化（无Eigen操作，类型安全）
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ompl_test_node");
    node->declare_parameter("turning_radius", 3.0);  // 声明参数，便于后续调整
    double turning_radius;
    node->get_parameter("turning_radius", turning_radius);

    // 2. 创建发布订阅器（纯ROS消息操作，无类型问题）
    auto gridmap_sub = node->create_subscription<grid_map_msgs::msg::GridMap>(
        "/grid_map", 10, Gridmap_Callback);
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
        "rs_path_marker", 10);
    path_vis_pub = node->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);

    // 3. OMPL配置（全double类型参数，避免类型混用）
    // 初始化Dubins状态空间（转弯半径为double，符合车辆模型需求）
    auto space = std::make_shared<ompl::base::DubinsStateSpace>(turning_radius);
    // 设置位置边界（全double类型，与坐标精度匹配）
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -10.0);   // X轴下界
    bounds.setHigh(0, 10.0);   // X轴上界
    bounds.setLow(1, -10.0);   // Y轴下界
    bounds.setHigh(1, 10.0);   // Y轴上界
    space->setBounds(bounds);

    // 初始化SimpleSetup，绑定状态有效性检查
    ompl::geometric::SimpleSetup ss(space);
    ss.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));

    // 4. 设置起点终点（全double类型，与Dubins状态空间匹配）
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::DubinsStateSpace::StateType>()->setX(-5.0);  // X坐标（double）
    start->as<ompl::base::DubinsStateSpace::StateType>()->setY(0.0);   // Y坐标（double）
    start->as<ompl::base::DubinsStateSpace::StateType>()->setYaw(0.0); // 航向角（double）

    ompl::base::ScopedState<> goal(space);
    goal->as<ompl::base::DubinsStateSpace::StateType>()->setX(5.0);   // X坐标（double）
    goal->as<ompl::base::DubinsStateSpace::StateType>()->setY(0.0);    // Y坐标（double）
    goal->as<ompl::base::DubinsStateSpace::StateType>()->setYaw(0.0);  // 航向角（double）

    ss.setStartAndGoalStates(start, goal);

    // 5. 设置路径优化目标（路径长度最小化，无类型问题）
    auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(
        ss.getSpaceInformation());
    ss.setOptimizationObjective(objective);

    // 6. 主循环（逻辑不变，确保所有数值操作均为double）
    rclcpp::Rate rate(10);  // 10Hz循环，符合局部规划实时性需求
    while (rclcpp::ok()) {
        // 仅当地图就绪时执行规划
        if (map_.exists("elevation")) {
            // 求解规划（1秒超时，避免阻塞主循环）
            const auto solved = ss.solve(1.0);
            if (solved) {
                RCLCPP_INFO(node->get_logger(), "Path found! Interpolating to 50 points");
                nav_msgs::msg::Path path_msg;
                path_msg.header.frame_id = "map";
                path_msg.header.stamp = node->get_clock()->now();

                // 路径插值（提升可视化平滑度，无类型问题）
                auto path = ss.getSolutionPath();
                path.interpolate(50);  // 插值为50个点，平衡精度与效率

                // 转换OMPL路径为ROS Path消息
                for (const auto& state : path.getStates()) {
                    const auto* dubins_state = state->as<ompl::base::DubinsStateSpace::StateType>();
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = path_msg.header;

                    // 位置赋值（全double类型）
                    pose.pose.position.x = dubins_state->getX();
                    pose.pose.position.y = dubins_state->getY();
                    pose.pose.position.z = 0.0;  // 2D场景，z轴固定为0.0

                    // 航向角→四元数（TF2操作，无Eigen类型冲突）
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, dubins_state->getYaw());  // 仅绕z轴旋转
                    q.normalize();  // 标准化四元数，确保合法性
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();

                    path_msg.poses.push_back(pose);
                }

                // 发布路径和Marker
                path_vis_pub->publish(path_msg);
                
                visualization_msgs::msg::Marker marker;
                marker.header = path_msg.header;
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.1;  // 线宽0.1m，便于可视化
                marker.color.r = 1.0;  // 红色
                marker.color.a = 1.0;  // 不透明
                for (const auto& pose : path_msg.poses) {
                    marker.points.push_back(pose.pose.position);
                }
                marker_pub->publish(marker);

                // 清除当前解，准备下一次规划
                ss.clear();
                ss.setStartAndGoalStates(start, goal);
            } else {
                RCLCPP_WARN(node->get_logger(), "No valid path found (timeout 1s)");
            }
        } else {
            // 地图未就绪时，每秒打印一次等待日志（避免刷屏）
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, 
                "Waiting for grid map (missing 'elevation' layer)...");
        }

        // 处理回调队列（非阻塞）
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 资源清理
    rclcpp::shutdown();
    return 0;
}