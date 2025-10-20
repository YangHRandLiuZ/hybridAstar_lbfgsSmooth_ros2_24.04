cmake_minimum_required(VERSION 3.8)
project(hybrid_astar_searcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)





# Include header directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# Create library
add_library(${PROJECT_NAME}
  src/hybrid_astar.cpp
  src/smooth.cpp
  src/ReedsSheppPath.cpp
  src/dynamicvoronoi.cpp
  src/calculate_heuristic.cpp          
  src/bucketedqueue.cpp      
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros  
  grid_map_core
  grid_map_msgs
  nav_msgs
  visualization_msgs
  tf2_geometry_msgs  
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen
${OMPL_LIBRARIES}
)

# Install library and headers
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}  # 关键：定义导出目标名，与后续 ament_export_targets 对应
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include  # 可选：明确头文件路径，增强依赖传递
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_export_dependencies(ompl)
ament_export_dependencies(grid_map_core)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()  # 原有的ament_package()保留
