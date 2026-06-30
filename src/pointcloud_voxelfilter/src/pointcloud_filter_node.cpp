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

class PointCloudVoxelFilter : public rclcpp::Node
{
public:
    PointCloudVoxelFilter() : Node("pointcloud_filter_node")
    {
        // 参数声明
        this->declare_parameter("target_frame", "base_link");
        this->declare_parameter("voxel_size", 0.04);                // 体素降采样
        this->declare_parameter("cutoff_z_near", 0.3);              // Z 轴近端截断 (m)
        this->declare_parameter("cutoff_z_far", 5.0);               // Z 轴远端截断 (m)
        this->declare_parameter("enable_SOR", true);                // 是否启用统计滤波
        this->declare_parameter("SOR_mean_k", 30);                  // SOR 近邻数
        this->declare_parameter("SOR_stddev", 1.0);                 // SOR 标准差倍数

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10,
            std::bind(&PointCloudVoxelFilter::process_pointcloud, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/filtered_points_baselink", 10);

        RCLCPP_INFO(this->get_logger(), "点云预处理节点已启动: TF转换→Passthrough→SOR→VoxelGrid");
    }

private:
    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::string target_frame = this->get_parameter("target_frame").as_string();

        // ── 1. TF 坐标转换：camera → base_link ──
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        try {
            auto transform_stamped = tf_buffer_->lookupTransform(
                target_frame, msg->header.frame_id,
                tf2::TimePointZero, tf2::durationFromSec(0.1));
            tf2::doTransform(*msg, cloud_transformed, transform_stamped);
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "TF 转换失败: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_transformed, *cloud);
        if (cloud->empty()) return;

        // ── 2. Passthrough Z：截断过近/过远噪声 ──
        double near = this->get_parameter("cutoff_z_near").as_double();
        double far  = this->get_parameter("cutoff_z_far").as_double();

        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(near, far);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
        pass_z.filter(*cloud_z);

        // ── 3. Statistical Outlier Removal：去除随机飞点 ──
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
        bool sor_enable = this->get_parameter("enable_SOR").as_bool();
        if (sor_enable && !cloud_z->empty()) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_z);
            sor.setMeanK(this->get_parameter("SOR_mean_k").as_int());
            sor.setStddevMulThresh(this->get_parameter("SOR_stddev").as_double());
            sor.filter(*cloud_clean);
        } else {
            *cloud_clean = *cloud_z;
        }

        if (cloud_clean->empty()) return;

        // ── 4. VoxelGrid 降采样 ──
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud_clean);
        double leaf = this->get_parameter("voxel_size").as_double();
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*cloud_filtered);

        // ── 5. 发布 ──
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header.frame_id = target_frame;
        output_msg.header.stamp = this->get_clock()->now();
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