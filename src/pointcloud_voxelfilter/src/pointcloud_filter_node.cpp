#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// TF2
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointCloudVoxelFilter : public rclcpp::Node
{
public:
    PointCloudVoxelFilter() : Node("pointcloud_filter_node")
    {
        // 1. 参数声明
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("voxel_size", 0.04);

        // 2. 初始化 TF 缓存和监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 订阅与发布
        // 订阅 RealSense 仿真插件发布的点云
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&PointCloudVoxelFilter::process_pointcloud, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/filtered_points_baselink", 10);

        RCLCPP_INFO(this->get_logger(), "点云处理节点已启动 (针对仿真优化版)");
    }

private:
    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::string target_frame = this->get_parameter("target_frame").as_string();
        sensor_msgs::msg::PointCloud2 cloud_transformed;

        try {
            // --- 核心修复：使用 TimePointZero 解决时间戳不匹配 ---
            // 无论仿真时间是多少，这行代码都会去找最新的 Link_06 到 base_link 的变换
            auto transform_stamped = tf_buffer_->lookupTransform(
                target_frame, 
                msg->header.frame_id, 
                tf2::TimePointZero, 
                tf2::durationFromSec(0.1));

            // 执行坐标转换
            tf2::doTransform(*msg, cloud_transformed, transform_stamped);
        }
        catch (const tf2::TransformException & ex) {
            // 使用间隔 log，防止报错刷屏
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "TF 转换失败: %s", ex.what());
            return;
        }

        // --- 体素滤波 ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_transformed, *pcl_cloud);

        if (pcl_cloud->empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pcl_cloud);
        double leaf = this->get_parameter("voxel_size").as_double();
        sor.setLeafSize(leaf, leaf, leaf);
        sor.filter(*cloud_filtered);

        // --- 发布结果 ---
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header.frame_id = target_frame;
        output_msg.header.stamp = this->get_clock()->now(); // 使用当前仿真时钟时间
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudVoxelFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}