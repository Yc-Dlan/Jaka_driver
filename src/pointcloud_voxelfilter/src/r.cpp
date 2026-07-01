#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

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
        this->declare_parameter("cutoff_z_near", 0.3);
        this->declare_parameter("cutoff_z_far", 5.0);
        this->declare_parameter("enable_SOR", true);
        this->declare_parameter("SOR_mean_k", 30);
        this->declare_parameter("SOR_stddev", 1.0);

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

        RCLCPP_INFO(this->get_logger(), "真机点云预处理节点: TF->Passthrough->SOR->VoxelGrid");
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
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "TF error: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_transformed, *cloud);
        if (cloud->empty()) return;

        // 2. Passthrough Z
        double near = this->get_parameter("cutoff_z_near").as_double();
        double far  = this->get_parameter("cutoff_z_far").as_double();
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(near, far);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
        pass_z.filter(*cloud_z);

        // 3. SOR
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
        if (this->get_parameter("enable_SOR").as_bool() && !cloud_z->empty()) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_z);
            sor.setMeanK(this->get_parameter("SOR_mean_k").as_int());
            sor.setStddevMulThresh(this->get_parameter("SOR_stddev").as_double());
            sor.filter(*cloud_clean);
        } else {
            *cloud_clean = *cloud_z;
        }
        if (cloud_clean->empty()) return;

        // 4. VoxelGrid
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_clean);
        double leaf = this->get_parameter("voxel_size").as_double();
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*cloud_filtered);

        // 5. publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header.frame_id = target_frame;
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