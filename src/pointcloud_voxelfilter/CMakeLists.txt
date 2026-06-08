cmake_minimum_required(VERSION 3.8)
project(pointcloud_voxelfilter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(PCL REQUIRED)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)

ament_target_dependencies(pointcloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
)

# 链接 PCL 依赖
target_link_libraries(pointcloud_filter_node
  ${PCL_LIBRARIES}
)

# ================= 新增核心部分：安装指令 =================
# 确保 ros2 run 能够找到编译好的可执行文件
install(TARGETS
  pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()