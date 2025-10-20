cmake_minimum_required(VERSION 3.8)
project(create_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)  # 新增：查找 Eigen3（版本≥3.3）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hybrid_astar_searcher REQUIRED)

# 【修正1：删除错误的 OMPL_DIR 路径（源码编译的 OMPL 不在 ROS 目录下）】
# set(OMPL_DIR "/opt/ros/jazzy/share/ompl/cmake")  # 注释或删除此行
find_package(ompl REQUIRED)  # 自动查找源码编译的 OMPL

# Include header directories
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}  # 保留：自动获取 OMPL 头文件路径（无需版本号）
  ${EIGEN3_INCLUDE_DIRS}  # 新增：自动获取 Eigen3 头文件路径
  # /usr/local/include/ompl-1.6  # 删除：移除版本化硬编码路径
  /usr/local/include
)

# Create executables
add_executable(simple_demo_node src/simple_demo_node.cpp)
add_executable(ompl_test_node src/ompl_test.cpp)
add_executable(hybrid_astar_test src/hybrid_astar_test.cpp)

# Link dependencies
ament_target_dependencies(simple_demo_node
  rclcpp
  geometry_msgs
  grid_map_ros
  grid_map_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(ompl_test_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  grid_map_ros
  grid_map_msgs
  tf2
  tf2_ros
  hybrid_astar_searcher
  # 【修正2：删除此处的 ompl，避免重复依赖】
)

ament_target_dependencies(hybrid_astar_test
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  grid_map_ros
  grid_map_msgs
  tf2
  tf2_ros
  hybrid_astar_searcher
  # 【修正2：删除此处的 ompl】
)

# 【修正3：用 OMPL_LIBRARIES 链接，而非 ompl::ompl（源码编译的 OMPL 不生成该目标）】
# target_link_libraries(ompl_test_node ${OMPL_LIBRARIES})
# target_link_libraries(hybrid_astar_test ${OMPL_LIBRARIES})
target_link_libraries(ompl_test_node ${OMPL_LIBRARIES} Eigen3::Eigen)  # 新增 Eigen3::Eigen
target_link_libraries(hybrid_astar_test ${OMPL_LIBRARIES} Eigen3::Eigen)  # 新增 Eigen3::Eigen

# Install executables
install(TARGETS
  simple_demo_node
  ompl_test_node
  hybrid_astar_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()