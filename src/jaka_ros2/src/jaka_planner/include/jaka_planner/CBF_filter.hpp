/**
 * 文件名: cbf_filter.hpp
 * 位置: jaka_planner/include/jaka_planner/cbf_filter.hpp
 * 功能: 封装 CBF 避障算法与可视化 (修复 Link 名称版)
 */

#ifndef CBF_FILTER_HPP
#define CBF_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class CBFFilter {
public:
    // === 参数配置 ===
    struct Config {
        double robot_radius = 0.08;  // 机械臂胶囊半径
        double obs_radius = 0.15;    // 障碍物半径
        Eigen::Vector3d obs_pos = {0.8, 0.0, 0.4}; // 障碍物位置
        double gamma = 5.0;          // 刹车激进程度
        double safe_dist_threshold = 0.20; // 激活距离
        std::string base_frame = "base_link";
    } cfg_;

    // === 初始化 ===
    void init(rclcpp::Node::SharedPtr node) {
        node_ = node;
        vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cbf_debug_markers", 10);
        RCLCPP_INFO(node_->get_logger(), "🛡️ CBF 安全滤波器已加载 (Obs: [%.1f, %.1f, %.1f])", 
            cfg_.obs_pos.x(), cfg_.obs_pos.y(), cfg_.obs_pos.z());
    }

    // === 核心接口：输入原始速度，原地修改为安全速度 ===
    bool filter(moveit::core::RobotStatePtr robot_state, 
                const moveit::core::JointModelGroup* jmg, 
                Eigen::VectorXd& q_dot_cmd) 
    {
        // 1. 定义胶囊体 (Link Name 必须正确，用于 Jacobian 计算)
        struct Capsule { Eigen::Vector3d p1, p2; std::string link_id; };
        std::vector<Capsule> capsules;
        
        auto getPos = [&](const std::string& link) {
            return robot_state->getGlobalLinkTransform(link).translation();
        };

        // 🛑 [关键修复] 使用 Link_0X (带前导零)
        try {
            capsules.push_back({getPos("Link_02"), getPos("Link_03"), "Link_03"}); // 上臂
            capsules.push_back({getPos("Link_03"), getPos("Link_05"), "Link_05"}); // 前臂 (忽略 Link_04)
            capsules.push_back({getPos("Link_05"), getPos("Link_06"), "Link_06"}); // 手腕
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "URDF Link 名称查询失败: %s", e.what());
            return false;
        }

        // 2. 寻找最近点
        double min_h = 999.0;
        Eigen::Vector3d closest_robot; // 已移除 closest_obs 以消除警告
        std::string critical_link;

        for (const auto& cap : capsules) {
            Eigen::Vector3d pt_on_seg;
            distPointToSegment(cfg_.obs_pos, cap.p1, cap.p2, pt_on_seg);
            double dist = (pt_on_seg - cfg_.obs_pos).norm();
            double h = dist - (cfg_.robot_radius + cfg_.obs_radius); // 表面距离

            if (h < min_h) {
                min_h = h;
                closest_robot = pt_on_seg;
                critical_link = cap.link_id;
            }
        }

        // 3. 可视化 (无论是否介入都画出来，方便调试)
        publishViz(closest_robot, min_h);

        // 4. 安全判断
        if (min_h > cfg_.safe_dist_threshold) return false; // 安全，无需介入

        // 5. 计算梯度与修正
        // 梯度方向 n = (Robot - Obs) / |dist| (远离障碍物的方向)
        Eigen::Vector3d n = (closest_robot - cfg_.obs_pos).normalized();

        // 转换到局部坐标系求雅可比
        const Eigen::Isometry3d& tf_link = robot_state->getGlobalLinkTransform(critical_link);
        Eigen::Vector3d local_pt = tf_link.inverse() * closest_robot;

        Eigen::MatrixXd J;
        if (!robot_state->getJacobian(jmg, robot_state->getLinkModel(critical_link), local_pt, J)) {
            return false;
        }

        // 提取线速度相关的 Jacobian (前3行)
        Eigen::MatrixXd J_lin = J.topRows(3);
        
        // 投影: v_danger = n^T * J * q_dot
        // CBF 约束: v_danger >= -gamma * h
        double v_danger = n.dot(J_lin * q_dot_cmd);
        double limit = -cfg_.gamma * min_h;

        if (v_danger < limit) {
            // 需要修正
            // 公式: q_safe = q - lambda * (n^T * J)^T
            Eigen::VectorXd J_n = (n.transpose() * J_lin).transpose(); // 6x1 向量
            
            double lambda = (limit - v_danger) / (J_n.dot(J_n) + 1e-6);
            q_dot_cmd = q_dot_cmd + lambda * J_n;
            
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, 
                "🛑 CBF 介入! Dist: %.3fm, Limit: %.3f", min_h, limit);
            return true;
        }

        return false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;

    // 数学辅助: 点到线段距离
    double distPointToSegment(const Eigen::Vector3d& p, 
                              const Eigen::Vector3d& a, 
                              const Eigen::Vector3d& b, 
                              Eigen::Vector3d& closest) 
    {
        Eigen::Vector3d ab = b - a;
        double t = (p - a).dot(ab) / ab.squaredNorm();
        t = std::max(0.0, std::min(1.0, t));
        closest = a + t * ab;
        return (p - closest).norm();
    }

    // 可视化辅助
    void publishViz(const Eigen::Vector3d& p_robot, double h) {
        visualization_msgs::msg::MarkerArray markers;
        auto now = node_->now();

        // Marker 1: 障碍物
        visualization_msgs::msg::Marker obs;
        obs.header.frame_id = cfg_.base_frame; obs.header.stamp = now;
        obs.ns = "cbf_obs"; obs.id = 0; obs.type = visualization_msgs::msg::Marker::SPHERE;
        obs.action = visualization_msgs::msg::Marker::ADD;
        obs.pose.position.x = cfg_.obs_pos.x(); obs.pose.position.y = cfg_.obs_pos.y(); obs.pose.position.z = cfg_.obs_pos.z();
        obs.scale.x = cfg_.obs_radius*2; obs.scale.y = cfg_.obs_radius*2; obs.scale.z = cfg_.obs_radius*2;
        obs.color.a = 0.4; obs.color.r = 1.0; obs.color.g = 0.0; obs.color.b = 0.0; // Red
        markers.markers.push_back(obs);

        // Marker 2: 连接线 (变色警告)
        visualization_msgs::msg::Marker line;
        line.header.frame_id = cfg_.base_frame; line.header.stamp = now;
        line.ns = "cbf_dist"; line.id = 1; line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.scale.x = 0.015; 
        
        // 颜色随距离变化: 安全(绿) -> 危险(红)
        line.color.a = 1.0;
        line.color.r = (h < 0.05) ? 1.0 : 0.0;
        line.color.g = (h > 0.1) ? 1.0 : 0.0;
        line.color.b = 0.0;

        geometry_msgs::msg::Point p1, p2;
        p1.x = cfg_.obs_pos.x(); p1.y = cfg_.obs_pos.y(); p1.z = cfg_.obs_pos.z();
        p2.x = p_robot.x(); p2.y = p_robot.y(); p2.z = p_robot.z();
        line.points.push_back(p1); line.points.push_back(p2);
        markers.markers.push_back(line);

        vis_pub_->publish(markers);
    }
};

#endif // CBF_FILTER_HPP