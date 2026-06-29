#include "rclcpp/rclcpp.hpp"
#include "Fiesta.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // ROS 2 推荐使用 SharedPtr 管理节点
  auto node = std::make_shared<rclcpp::Node>("FIESTA");
  //fiesta::Fiesta<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> esdf_map(node);
  fiesta::Fiesta<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped> fiesta_node(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}