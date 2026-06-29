cmake_minimum_required(VERSION 3.8)
project(fiesta)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -pthread")
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)     # ROS 2 中 tf 被替换为 tf2_ros
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)         # 去掉 1.7 版本硬编码，使用系统默认高版本
find_package(OpenCV REQUIRED)


include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})


add_library(fiesta_lib
    src/parameters.cpp
    src/timing.cc
    src/raycast.cpp
    src/ESDFMap.cpp
)
ament_target_dependencies(fiesta_lib
    rclcpp
    sensor_msgs
    std_msgs
    nav_msgs
    visualization_msgs
    tf2_ros
    cv_bridge
    pcl_conversions
)
target_link_libraries(fiesta_lib
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)


add_executable(test_fiesta
    test/test_fiesta.cpp
)
# 链接刚才生成的库
target_link_libraries(test_fiesta
    fiesta_lib
)
ament_target_dependencies(test_fiesta
    rclcpp
    sensor_msgs
    std_msgs
    nav_msgs
    visualization_msgs
    tf2_ros
    cv_bridge
    pcl_conversions
)


install(TARGETS
    fiesta_lib
    test_fiesta
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

# 安装 Python 脚本
install(PROGRAMS scripts/static_pose_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

# 将 launch 文件夹和 rviz 配置安装到 share 目录下
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES demo.rviz
    DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_libraries(fiesta_lib)
ament_export_dependencies(
    rclcpp 
    sensor_msgs
    std_msgs 
    nav_msgs 
    visualization_msgs 
    tf2_ros 
    cv_bridge
)

ament_package()