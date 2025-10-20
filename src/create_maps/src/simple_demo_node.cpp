/**
 * @file simple_demo_node.cpp
 * @author jiaxier
 * @brief ROS 2 port of grid map demo
 * @version 0.1
 * @date 2023-12-23
 */
#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <cmath>

using namespace grid_map;

int main(int argc, char **argv)
{
    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("grid_map_simple_demo");
    
    // 创建发布者（ROS 2使用create_publisher）
    auto map_pub = node->create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", 10);

    // 创建网格地图
    GridMap map({"elevation", "traversability"});
    map.setFrameId("map");
    map.setGeometry(Length(50, 50), 1);  // 尺寸50x50m，分辨率1m
    RCLCPP_INFO(node->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
                map.getLength().x(), map.getLength().y(),
                map.getSize()(0), map.getSize()(1));

    // 定义障碍物多边形
    Polygon polygon1({{-8, 0}, {-8, 5}, {-3, 5}, {-3, 0}});
    Polygon polygon2({{13, 12}, {6, 12}, {10, 4}});
    Polygon polygon3({{1, -10}, {1, -5}, {6, -5}, {6, -10}});
    Polygon polygon4({{0, -20}, {0, -25}, {5, -25}, {5, -20}});
    Polygon polygon5({{-11, 16}, {-15, 9}, {-22, 12}, {-21, 20}, {-14, 23}});
    Polygon polygon6({{-13, -4}, {-8, -12}, {-13, -20}, {-21, -5}});
    Polygon polygon7({{2, 16}, {-4, 20}, {-4, 10}});
    Polygon polygon8({{25, 20}, {25, 19}, {5, 19}, {5, 20}});
    Polygon polygon9({{25, 17}, {25, 16}, {5, 16}, {5, 17}});
    Polygon polygon10({{22, 2}, {17, 5}, {12, 2}, {17, -1}});

    rclcpp::Rate rate(10);  // 10Hz循环
    while (rclcpp::ok()) {
        // 重置高程为0
        for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 0;
        }

        // 设置多边形区域为障碍物（高程1）
        std::vector<Polygon> polygons = {polygon1, polygon2, polygon3, polygon4, polygon5,
                                         polygon6, polygon7, polygon8, polygon9, polygon10};
        for (const auto& poly : polygons) {
            for (PolygonIterator it(map, poly); !it.isPastEnd(); ++it) {
                map.at("elevation", *it) = 1;
            }
        }

        // 添加圆形障碍物
        for (CircleIterator it(map, {16, -13}, 5); !it.isPastEnd(); ++it) {
            map.at("elevation", *it) = 1;
        }

        // 发布地图消息
// ROS 2 版本：通过返回值获取消息指针并发布
auto msg_ptr = GridMapRosConverter::toMessage(map);
if (msg_ptr) {  // 确保消息生成成功
    msg_ptr->header.stamp = node->get_clock()->now();  // 设置时间戳（如果需要）
    map_pub->publish(*msg_ptr);
}
        rate.sleep();
        rclcpp::spin_some(node);  // 处理回调（此处无回调，可选）
    }

    rclcpp::shutdown();
    return 0;
}