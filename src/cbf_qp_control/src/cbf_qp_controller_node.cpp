#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

using namespace Eigen;

class CbfQpControllerNode : public rclcpp::Node
{
public:
    CbfQpControllerNode() : Node("cbf_qp_controller_node")
    {
        // 初始化 OSQP 求解器参数
        solver_.settings()->setVerbosity(false);
        solver_.settings()->setWarmStart(true);
        solver_.data()->setNumberOfVariables(6); // 6自由度机械臂
        solver_.data()->setNumberOfConstraints(9); // 6个关节限位 + 1个ESDF + 1个自碰撞 + 1个奇异点

        // 分配内存
        hessian_.resize(6, 6);
        hessian_.setIdentity(); // P = I
        gradient_.resize(6);
        linearMatrix_.resize(9, 6);
        lowerBound_.resize(9);
        upperBound_.resize(9);

        // 初始化 Hessian 矩阵 (由于 P=I，为常数矩阵，只需设置一次)
        solver_.data()->setHessianMatrix(hessian_);

        // 订阅标称指令 (遥操单元下发) 与真实关节状态
        sub_nom_cmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/nominal_joint_velocity", 10, std::bind(&CbfQpControllerNode::nominalCmdCallback, this, std::placeholders::_1));
            
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&CbfQpControllerNode::jointStateCallback, this, std::placeholders::_1));

        // 发布安全过滤后的实际指令
        pub_safe_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/safe_joint_velocity", 10);

        RCLCPP_INFO(this->get_logger(), "动态多约束 CBF-QP 安全滤波单元已启动");
    }

private:
    void nominalCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 6 || current_q_.size() != 6) return;

        // 1. 获取前端传入的标称关节速度指令 u_nom
        VectorXd u_nom = Map<const VectorXd>(msg->data.data(), 6);
        
        // 目标函数的梯度向量 q = -u_nom
        gradient_ = -u_nom;

        // 2. 构建约束矩阵 A, l, u
        updateConstraints();

        // 3. 将更新后的数据传入求解器
        if (!is_solver_initialized_) {
            solver_.data()->setGradient(gradient_);
            solver_.data()->setLinearConstraintsMatrix(linearMatrix_);
            solver_.data()->setLowerBound(lowerBound_);
            solver_.data()->setUpperBound(upperBound_);
            solver_.initSolver();
            is_solver_initialized_ = true;
        } else {
            solver_.updateGradient(gradient_);
            solver_.updateLinearConstraintsMatrix(linearMatrix_);
            solver_.updateBounds(lowerBound_, upperBound_);
        }

        // 4. 求解 QP 问题
        if (solver_.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
            VectorXd u_safe = solver_.getSolution();
            
            // 发布最优安全指令 u
            std_msgs::msg::Float64MultiArray safe_msg;
            safe_msg.data.assign(u_safe.data(), u_safe.data() + u_safe.size());
            pub_safe_cmd_->publish(safe_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "CBF-QP 求解失败！机械臂触发硬刹车！");
            // 兜底机制：发送全 0 速度
            std_msgs::msg::Float64MultiArray stop_msg;
            stop_msg.data.resize(6, 0.0);
            pub_safe_cmd_->publish(stop_msg);
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 实时更新当前关节位置 q
        current_q_ = Map<const VectorXd>(msg->position.data(), 6);
    }

    void updateConstraints()
    {
        linearMatrix_.setZero();
        
        // ---------------------------------------------------------
        // 约束 1：关节物理极限 (前 6 行)
        // 公式: u >= \eta(q_min - q) 且 -u >= \eta(q - q_max)
        // ---------------------------------------------------------
        double eta = 5.0; // 趋近速度衰减系数
        VectorXd q_min = VectorXd::Constant(6, -3.14); // 替换为真实下限
        VectorXd q_max = VectorXd::Constant(6, 3.14);  // 替换为真实上限
        
        linearMatrix_.block(0, 0, 6, 6) = MatrixXd::Identity(6, 6);
        lowerBound_.segment(0, 6) = eta * (q_min - current_q_);
        upperBound_.segment(0, 6) = -eta * (current_q_ - q_max);

        // ---------------------------------------------------------
        // 约束 2：动态 ESDF 防撞 (第 7 行)
        // 公式: \nabla d_{ESDF}^T J_{pq} u >= -\gamma_v (d_{ESDF} - d_{safe})
        // ---------------------------------------------------------
        // TODO: 从你的点云距离节点中获取以下变量
        Vector3d grad_d_esdf(1.0, 0.0, 0.0); // 距离场梯度向量
        MatrixXd J_pq = MatrixXd::Zero(3, 6); // 胶囊体最近点的局部雅可比
        double d_esdf = 0.2; // 当前算出的最短距离
        double d_safe = 0.05; // 绝对安全距离阈值
        
        // 计算动态调节函数 \gamma_v = \gamma_0 * e^{-\alpha v}
        double current_v = 0.1; // 当前末端线速度
        double gamma_0 = 2.0;
        double alpha = 1.0;
        double gamma_v = gamma_0 * std::exp(-alpha * current_v);

        linearMatrix_.row(6) = grad_d_esdf.transpose() * J_pq;
        lowerBound_(6) = -gamma_v * (d_esdf - d_safe);
        upperBound_(6) = OsqpEigen::INFTY;

        // ---------------------------------------------------------
        // 约束 3：自碰撞避让 (第 8 行)
        // 公式: \nabla d_{self}^T J_{self}(q) u >= -\gamma_{self}(d_{self} - d_{self_safe})
        // ---------------------------------------------------------
        Vector3d grad_d_self(0.0, 1.0, 0.0); 
        MatrixXd J_self = MatrixXd::Zero(3, 6);
        double d_self = 0.15;
        double d_self_safe = 0.02;
        double gamma_self = 2.0;

        linearMatrix_.row(7) = grad_d_self.transpose() * J_self;
        lowerBound_(7) = -gamma_self * (d_self - d_self_safe);
        upperBound_(7) = OsqpEigen::INFTY;

        // ---------------------------------------------------------
        // 约束 4：奇异点规避 (第 9 行)
        // 公式: \partial w / \partial q u >= -\alpha_w (w(q) - w_{min})
        // ---------------------------------------------------------
        VectorXd grad_w = VectorXd::Zero(6); // 可操作度梯度
        double w_q = 0.05; // 当前 Yoshikawa 指标
        double w_min = 0.01;
        double alpha_w = 1.0;

        linearMatrix_.row(8) = grad_w.transpose();
        lowerBound_(8) = -alpha_w * (w_q - w_min);
        upperBound_(8) = OsqpEigen::INFTY;
    }

    OsqpEigen::Solver solver_;
    bool is_solver_initialized_ = false;

    SparseMatrix<double> hessian_;
    VectorXd gradient_;
    SparseMatrix<double> linearMatrix_;
    VectorXd lowerBound_;
    VectorXd upperBound_;
    VectorXd current_q_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_nom_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_safe_cmd_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CbfQpControllerNode>());
    rclcpp::shutdown();
    return 0;
}