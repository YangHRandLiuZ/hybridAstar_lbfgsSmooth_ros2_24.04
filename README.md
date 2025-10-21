本项目基于 https://github.com/USE-jx/hybridAstar_lbfgsSmooth
项目进行修改
原项目是在20.04以及ros1 melodic上进行实现
由于项目需求，将其进行了修改，使其在24.04以及ros2 jazzy上能够正常跑通

原项目readme：
hybridAstar_lbfgsSmooth  hybridAstar_lbfgs 平滑
hybrid astar with smooth, optimization solver is lbfgs
混合 Astar，具有平滑功能，优化求解器为 lbfgs

运行效果
image-20231222222329069


安装依赖
1 安装grid_map  1 安装 grid_map

sudo apt install ros-$ROS_DISTRO-grid-map
2 安装OMPL运动规划库

sydo apt install ros-$ROS_DISTRO-ompl
编译运行
创建一个工作空间

把src放入工作空间后catkin_make编译即可.
