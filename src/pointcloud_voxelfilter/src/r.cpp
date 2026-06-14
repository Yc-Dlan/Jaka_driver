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

class PointCloudRealWorldProcessor : public rclcpp::Node
{
public:
    PointCloudRealWorldProcessor() : Node("pointcloud_filter_node")
    {
        // 1. 参数声明（把输入话题也抽成参数，方便适配真实的 RealSense 命名空间）
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("voxel_size", 0.04);
        this->declare_parameter("input_topic", "/camera/depth/color/points"); 

        std::string input_topic = this->get_parameter("input_topic").as_string();

        // 2. 初始化 TF 缓存和监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 订阅真实相机话题
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&PointCloudRealWorldProcessor::process_pointcloud, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/filtered_points_baselink", 10);

        RCLCPP_INFO(this->get_logger(), "真实硬件场景点云处理节点已启动。运行于系统硬时钟。");
    }

private:
    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::string target_frame = this->get_parameter("target_frame").as_string();
        sensor_msgs::msg::PointCloud2 cloud_transformed;

        try {
            // --- 核心修改 1：严格的时间戳对齐 ---
            // 使用 msg->header.stamp，查找“这帧点云被捕获的精确时刻”机械臂的 TF 姿态。
            // 容忍度设为 50ms (0.05s)，给硬件网络传输留出时间。
            auto transform_stamped = tf_buffer_->lookupTransform(
                target_frame, 
                msg->header.frame_id, 
                msg->header.stamp,          // ⬅️ 放弃 TimePointZero，使用点云原生时间戳
                tf2::durationFromSec(0.05)); // ⬅️ 等待 TF 缓存同步的超时时间

            // 执行精准的时间同步坐标转换
            tf2::doTransform(*msg, cloud_transformed, transform_stamped);
        }
        catch (const tf2::TransformException & ex) {
            // 真实场景中，刚开机或网络抖动时丢几帧 TF 很常见，使用限速警告防止刷屏
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "硬件时钟未完全对齐，等待变换: %s", ex.what());
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
        
        // --- 核心修改 2：保留原始时间戳 ---
        // 后续避障控制器（如 APF 或 MPC）需要知道这个点云是哪一个历史时刻的数据
        output_msg.header.stamp = msg->header.stamp; 
        
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
    auto node = std::make_shared<PointCloudRealWorldProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}