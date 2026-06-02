#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_msgs/srv/move.hpp"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include "jaka_msgs/srv/set_user_frame.hpp"
#include "jaka_msgs/srv/set_tcp_frame.hpp"
#include "jaka_msgs/srv/set_payload.hpp"
#include "jaka_msgs/srv/set_collision.hpp"
#include "jaka_msgs/srv/set_io.hpp"
#include "jaka_msgs/srv/get_io.hpp"
#include "jaka_msgs/srv/get_fk.hpp"
#include "jaka_msgs/srv/get_ik.hpp"
#include "jaka_msgs/srv/clear_error.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>
#include <map>
#include <chrono>
#include <thread>
using namespace std;

const double PI = 3.1415926;
int jog_index_last = -1; 
int jog_count = 0;
int jog_count_temp = 0;
JAKAZuRobot robot;

map<int, string>mapErr = {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
};

rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tool_position_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_position_pub;
rclcpp::Publisher<jaka_msgs::msg::RobotMsg>::SharedPtr robot_state_pub;

bool linear_move_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    CartesianPose end_pose;
    double speed = static_cast<double>(request->mvvelo);
    double accel = static_cast<double>(request->mvacc);
    double tol = 0.5;
    OptionalCond *option_cond = nullptr;
    end_pose.tran.x = request->pose[0];
    end_pose.tran.y = request->pose[1];
    end_pose.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3], request->pose[4], request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(end_pose.rpy));
    
    int ret = robot.linear_move(&end_pose, MoveMode::ABS, TRUE, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "linear_move has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool joint_move_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = request->pose[0];
    joint_pose.jVal[1] = request->pose[1];
    joint_pose.jVal[2] = request->pose[2];
    joint_pose.jVal[3] = request->pose[3];
    joint_pose.jVal[4] = request->pose[4]; 
    joint_pose.jVal[5] = request->pose[5];
    double speed = static_cast<double>(request->mvvelo);
    double accel = static_cast<double>(request->mvacc);
    double tol = 0.5;
    OptionalCond *option_cond = nullptr;

    int ret = robot.joint_move(&joint_pose, MoveMode::ABS, true, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "joint_move has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool jog_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    double move_velocity = 0;
    CoordType coord_type = COORD_JOINT;
    
    float index_temp = static_cast<float>(request->index) / 2 + 0.1;
    int index = static_cast<int>(index_temp);
    
    switch (request->coord_mode)
    {
        case 0: coord_type = COORD_JOINT; move_velocity = request->mvacc; break;
        case 1: coord_type = COORD_BASE; move_velocity = request->mvacc; break;
        case 2: coord_type = COORD_TOOL; move_velocity = request->mvacc; break; 
        default:
            RCLCPP_INFO(rclcpp::get_logger("jog_callback"), "Coordinate system input error, please re-enter");
            return true;
    }
    if(request->index & 1) { move_velocity = -move_velocity; }
    
    if (jog_index_last != request->index)
    {   
        int ret = robot.motion_abort();
        if (ret == 0)
        {
            int jog_state = robot.jog(index, CONTINUE, coord_type, move_velocity, 0);
            switch(jog_state)
            {
                case 0: response->ret = 1; response->message = "Position is reached"; break;
                default: response->ret = jog_state; response->message = "error occurred:" + mapErr[jog_state]; break;
            }
        }
        else { response->ret = ret; response->message = "error occurred:" + mapErr[ret]; }
        jog_index_last = request->index;
    }
    else
    {
        response->ret = 1; response->message = "Robot is jogging";
    }
    jog_count = jog_count + 1;
    return true;
}

bool servo_move_enable_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    BOOL enable = request->enable;
    int ret = robot.servo_move_enable(enable);
    switch(ret)
    {
        case 0: response->ret = 1; response->message = "servo_move_enable has been executed"; break;
        default: response->ret = 0; response->message = "error occurred:" + mapErr[ret]; return false;
    }
    return true;
}

bool stop_move_callback([[maybe_unused]] const shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] shared_ptr<std_srvs::srv::Empty::Response> response)
{
    jog_count = 0; jog_count_temp = 0; jog_index_last = -1;
    int ret = robot.motion_abort();
    switch(ret)
    {
        case 0: RCLCPP_INFO(rclcpp::get_logger("stop_move_callback"), "stop_move has been executed"); break;
        default: RCLCPP_INFO(rclcpp::get_logger("stop_move_callback"), "error occurred: %s", mapErr[ret].c_str()); return false;;
    }
    return true;
}

bool set_toolFrame_callback(const shared_ptr<jaka_msgs::srv::SetTcpFrame::Request> request,
    shared_ptr<jaka_msgs::srv::SetTcpFrame::Response> response)
{
    CartesianPose tool_frame;
    int tool_frame_id = request->tool_num;
    tool_frame.tran.x = request->pose[0]; tool_frame.tran.y = request->pose[1]; tool_frame.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3],request->pose[4],request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(tool_frame.rpy));

    int ret = robot.set_tool_data(tool_frame_id, &tool_frame, "ToolCoord");
    if(ret == 0) { response->ret = 1; response->message = "set_toolFrame has been executed"; }
    else { response->ret = 0; response->message = "error occurred:" + mapErr[ret]; return false; }
    return true;
}

bool set_userFrame_callback(const shared_ptr<jaka_msgs::srv::SetUserFrame::Request> request,
    shared_ptr<jaka_msgs::srv::SetUserFrame::Response> response)
{
    CartesianPose user_frame;
    int user_frame_id = request->user_num; 
    user_frame.tran.x = request->pose[0]; user_frame.tran.y = request->pose[1]; user_frame.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3],request->pose[4],request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(user_frame.rpy));
    
    int ret = robot.set_user_frame_data(user_frame_id, &user_frame, "BaseCoord");
    if(ret == 0) { response->ret = 1; response->message = "set_userFrame has been executed"; }
    else { response->ret = 0; response->message = "error occurred:" + mapErr[ret]; return false; }
    return true;
}

bool set_payload_callback(const shared_ptr<jaka_msgs::srv::SetPayload::Request> request,
    shared_ptr<jaka_msgs::srv::SetPayload::Response> response)
{
    PayLoad payload;
    int tool_id = request->tool_num;
    payload.centroid.x = request->xc; payload.centroid.y = request->yc; payload.centroid.z = request->zc;
    payload.mass = request->mass;

    robot.set_tool_id(tool_id);
    int ret = robot.set_payload(&payload);
    if(ret == 0) { response->ret = 1; response->message = "set_payload has been executed"; }
    else { response->ret = 0; response->message = "error occurred:" + mapErr[ret]; return false; }
    return true;
}

bool drag_mode_callback(const shared_ptr<std_srvs::srv::SetBool::Request> request,
    shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int ret = robot.drag_mode_enable(request->data);
    if(ret == 0) { response->success = 1; response->message = "drag_mode has been executed"; }
    else { response->success = 0; response->message = "error occurred:" + mapErr[ret]; return false; }
    return true;
}

bool set_collisionLevel_callback(const shared_ptr<jaka_msgs::srv::SetCollision::Request> request,
    shared_ptr<jaka_msgs::srv::SetCollision::Response> response)
{
    int collision_level;
    if(request->is_enable == 0) { collision_level = 0; }
    else {
        if(request->value <= 25) collision_level = 1;
        else if(request->value <= 50) collision_level = 2;
        else if(request->value <= 75) collision_level = 3;
        else if(request->value <=100) collision_level = 4;
        else collision_level = 5;
    }
    int ret = robot.set_collision_level(collision_level);
    if(ret == 0) { response->ret = 1; response->message = "Collision level" + to_string(collision_level) + " has been executed"; }
    else { response->ret = 0; response->message = "error occurred:" + mapErr[ret]; return false; }
    return true;
}

bool set_io_callback(const shared_ptr<jaka_msgs::srv::SetIO::Request> request,
    shared_ptr<jaka_msgs::srv::SetIO::Response> response)
{   
    IOType type; int ret;
    switch(request->type)
    {
        case 0: type = IO_CABINET; break;
        case 1: type = IO_TOOL; break;
        case 2: type = IO_EXTEND; break;
    }
    float value = request->value; string signal = request->signal; int index = request->index;
    
    if(signal == "digital") {      
        BOOL digital_value = value ? TRUE : FALSE;
        ret = robot.set_digital_output(type, index, digital_value);
    }
    else if(signal == "analog") {
        ret = robot.set_analog_output(type, index, value);
    }
    
    if(ret == 0) { response->ret = 1; response->message = "set IO has been executed"; }
    else { response->ret = 0; response->message = "error occurred:" + mapErr[ret]; return false; }
    return true;
}

bool get_io_callback(const shared_ptr<jaka_msgs::srv::GetIO::Request> request,
    shared_ptr<jaka_msgs::srv::GetIO::Response> response)
{   
    IOType type; int ret; BOOL digital_result; float analog_result;
    switch(request->type) {
        case 0: type = IO_CABINET; break;
        case 1: type = IO_TOOL; break;
        case 2: type = IO_EXTEND; break;
        default: response->value = -999999; response->message = "Invalid IO type"; return false;
    }
    
    string signal = request->signal; int index = request->index; int path = request->path;
    
    if(signal == "digital") {       
        if(path == 0) ret = robot.get_digital_input(type, index, &digital_result);
        else if(path == 1) ret = robot.get_digital_output(type, index, &digital_result);
        else { response->value = -999999; response->message = "Invalid path value"; return false; }
        
        if(ret == 0) { response->value = float(digital_result); response->message = "get IO has been executed"; }
        else { response->value = -999999; response->message = "error occurred:" + mapErr[ret]; }
        return true;
    }
    else if(signal == "analog") {
        if(path == 0) ret = robot.get_analog_input(type, index, &analog_result);
        else if(path == 1) ret = robot.get_analog_output(type, index, &analog_result);
        else { response->value = -999999; response->message = "Invalid path value"; return false; }
        
        if(ret == 0) { response->value = analog_result; response->message = "get IO has been executed"; }
        else { response->value = -999999; response->message = "error occurred:" + mapErr[ret]; }
        return true;
    }
    else {
        response->value = -999999; response->message = "Invalid signal type"; return false;
    }
    return true;
}

bool get_fk_callback(const shared_ptr<jaka_msgs::srv::GetFK::Request> request,
    shared_ptr<jaka_msgs::srv::GetFK::Response> response)
{
    JointValue joint_pose; CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++) { joint_pose.jVal[i] = request->joint[i]; }
    
    int ret = robot.kine_forward(&joint_pose, &cartesian_pose);
    if(ret == 0) {
        response->cartesian_pose.push_back(cartesian_pose.tran.x);
        response->cartesian_pose.push_back(cartesian_pose.tran.y);
        response->cartesian_pose.push_back(cartesian_pose.tran.z);
        response->cartesian_pose.push_back(cartesian_pose.rpy.rx);
        response->cartesian_pose.push_back(cartesian_pose.rpy.ry);
        response->cartesian_pose.push_back(cartesian_pose.rpy.rz);
        response->message = "get FK has been executed";
    } else {
        float pose_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
        for(int i = 0; i < 6; i++) { response->cartesian_pose.push_back(pose_init[i]); }
        response->message = "error occurred:" + mapErr[ret];
        return false;
    }
    return true;
}

bool get_ik_callback(const shared_ptr<jaka_msgs::srv::GetIK::Request> request,
    shared_ptr<jaka_msgs::srv::GetIK::Response> response)
{
    JointValue joint_pose; JointValue ref_joint; CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++) { ref_joint.jVal[i] = request->ref_joint[i]; }
    
    cartesian_pose.tran.x = request->cartesian_pose[0];
    cartesian_pose.tran.y = request->cartesian_pose[1];
    cartesian_pose.tran.z = request->cartesian_pose[2];
    cartesian_pose.rpy.rx = request->cartesian_pose[3];
    cartesian_pose.rpy.ry = request->cartesian_pose[4];
    cartesian_pose.rpy.rz = request->cartesian_pose[5];
    
    int ret = robot.kine_inverse(&ref_joint, &cartesian_pose, &joint_pose);
    if(ret == 0) {
        for(int i = 0; i < 6; i++) { response->joint.push_back(joint_pose.jVal[i]); }
        response->message = "get IK has been executed";
    } else {
        float joint_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
        for(int i = 0; i < 6; i++) { response->joint.push_back(joint_init[i]); }
        response->message = "error occurred:" + mapErr[ret];
        return false;
    }
    return true;
}

void tool_position_callback(const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& tool_position_pub)
{
    if (!tool_position_pub) {
        RCLCPP_ERROR(rclcpp::get_logger("jaka_driver"), "Publisher is not initialized!");
        return;
    }
    geometry_msgs::msg::TwistStamped tool_position;
    CartesianPose tcp_position;
    RotMatrix rot; Rpy rpy;
    
    robot.get_tcp_position(&tcp_position);
    tool_position.twist.linear.x = tcp_position.tran.x;
    tool_position.twist.linear.y = tcp_position.tran.y;
    tool_position.twist.linear.z = tcp_position.tran.z;
    rpy.rx = tcp_position.rpy.rx;
    rpy.ry = tcp_position.rpy.ry;
    rpy.rz = tcp_position.rpy.rz;

    robot.rpy_to_rot_matrix(&rpy, &rot);
    
    tool_position.twist.angular.x = (rpy.rx )/PI*180;
    tool_position.twist.angular.y = (rpy.ry )/PI*180;
    tool_position.twist.angular.z = (rpy.rz )/PI*180;
    
    tool_position.header.stamp = rclcpp::Clock().now();
    tool_position_pub->publish(tool_position);
}

void joint_position_callback(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_position_pub)
{
    sensor_msgs::msg::JointState joint_position;
    JointValue joint_pos;
    robot.get_joint_position(&joint_pos);
    
    for (int i = 0; i < 6; i++) {
        joint_position.position.push_back(joint_pos.jVal[i]); 
        joint_position.name.push_back("joint_" + to_string(i + 1));
    }
    joint_position.header.stamp = rclcpp::Clock().now();
    joint_position_pub->publish(joint_position);
}

void robot_states_callback(const rclcpp::Publisher<jaka_msgs::msg::RobotMsg>::SharedPtr& robot_states_pub)
{
    jaka_msgs::msg::RobotMsg robot_states;
    RobotStatus_simple robotstatus_simple;
    ProgramState programstate;
    BOOL in_pos = true, in_col = false, drag_mode = false, emergency_stop = false;
    
    robot.is_in_pos(&in_pos); robot.is_in_collision(&in_col);
    robot.is_in_drag_mode(&drag_mode); robot.is_in_estop(&emergency_stop);
    robot.get_robot_status_simple(&robotstatus_simple);
    robot.get_program_state(&programstate);

    if(emergency_stop) robot_states.motion_state = 2;
    else if(robotstatus_simple.errcode) robot_states.motion_state = 4;
    else if(in_pos && programstate == PROGRAM_IDLE && (!drag_mode)) robot_states.motion_state = 0;
    else if(programstate == PROGRAM_PAUSED) robot_states.motion_state = 1;
    else if((!in_pos) || programstate == PROGRAM_RUNNING || drag_mode) robot_states.motion_state = 3;

    if(robotstatus_simple.powered_on) robot_states.power_state = 1;
    else robot_states.power_state = 0;

    if(robotstatus_simple.enabled) robot_states.servo_state = 1;
    else robot_states.servo_state = 0;

    if(in_col) robot_states.collision_state = 1;
    else robot_states.collision_state = 0;
    
    robot_states_pub->publish(robot_states);
}

void stop_jog_callback()
{
     if (jog_count >= 1 && jog_count_temp == jog_count ) {
        robot.jog_stop(-1);
        jog_count = 0; jog_count_temp = 0; jog_index_last = -1;
        RCLCPP_INFO(rclcpp::get_logger("stop_jog_callback"), "jog stop");
    }
    jog_count_temp = jog_count;
}

void get_conn_scoket_state(){
    JointValue temp_joints;
    while (rclcpp::ok()) {
        int ret = robot.get_joint_position(&temp_joints);
        if (ret) {
            RCLCPP_ERROR(rclcpp::get_logger("get_conn_socket_state"), 
                         "Connection error or get_joint_position failed, error_code: %d, error: %s", ret, mapErr[ret].c_str());
        }
        if(ret==0) {
            tool_position_callback(tool_position_pub);
            joint_position_callback(joint_position_pub);
            robot_states_callback(robot_state_pub);
        }
        rclcpp::sleep_for(chrono::milliseconds(100)); 
    }    
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jaka_driver");
    rclcpp::Rate rate(125); 
    
    string default_ip = "10.5.5.100";
    string robot_ip = node->declare_parameter("ip", default_ip);
    robot.login_in(robot_ip.c_str(), false);
    robot.set_status_data_update_time_interval(100);
    robot.set_block_wait_timeout(120);
    robot.power_on();
    sleep(8);
    robot.enable_robot();
    sleep(4);
    
    robot.servo_move_use_joint_LPF(0.1);
    robot.servo_speed_foresight(15,0.03);

    auto linear_move_service = node->create_service<jaka_msgs::srv::Move>("/jaka_driver/linear_move", &linear_move_callback);
    auto joint_move_service = node->create_service<jaka_msgs::srv::Move>("/jaka_driver/joint_move", &joint_move_callback);
    auto jog_service = node->create_service<jaka_msgs::srv::Move>("/jaka_driver/jog", &jog_callback);
    auto servo_move_enable_service = node->create_service<jaka_msgs::srv::ServoMoveEnable>("/jaka_driver/servo_move_enable", &servo_move_enable_callback);
    auto stop_move_service = node->create_service<std_srvs::srv::Empty>("/jaka_driver/stop_move", &stop_move_callback);
    auto set_toolframe_service = node->create_service<jaka_msgs::srv::SetTcpFrame>("/jaka_driver/set_toolframe", &set_toolFrame_callback);
    auto set_userframe_service = node->create_service<jaka_msgs::srv::SetUserFrame>("/jaka_driver/set_userframe", &set_userFrame_callback);
    auto set_payload_service = node->create_service<jaka_msgs::srv::SetPayload>("/jaka_driver/set_payload", &set_payload_callback);
    auto drag_move_service = node->create_service<std_srvs::srv::SetBool>("/jaka_driver/drag_move", &drag_mode_callback);
    auto set_collisionlevel_service = node->create_service<jaka_msgs::srv::SetCollision>("/jaka_driver/set_collisionlevel", &set_collisionLevel_callback);
    auto set_io_service = node->create_service<jaka_msgs::srv::SetIO>("jaka_driver/set_io",&set_io_callback);
    auto get_io_service = node->create_service<jaka_msgs::srv::GetIO>("jaka_driver/get_io",&get_io_callback);
    auto get_fk_service = node->create_service<jaka_msgs::srv::GetFK>("jaka_driver/get_fk", &get_fk_callback);
    auto get_ik_service = node->create_service<jaka_msgs::srv::GetIK>("jaka_driver/get_ik", &get_ik_callback);

    tool_position_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/jaka_driver/tool_position", 10);
    joint_position_pub = node->create_publisher<sensor_msgs::msg::JointState>("/jaka_driver/joint_position", 10);
    robot_state_pub = node->create_publisher<jaka_msgs::msg::RobotMsg>("/jaka_driver/robot_states", 10);
    
    // [核心修改] 新增的高频伺服 Topic 订阅 (替换原有的 RPC 服务)
    auto servo_p_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/jaka_driver/servo_p_cmd", rclcpp::SensorDataQoS(),
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() < 6) return;
            CartesianPose cartesian_pose;
            cartesian_pose.tran.x = msg->data[0];
            cartesian_pose.tran.y = msg->data[1];
            cartesian_pose.tran.z = msg->data[2];
            cartesian_pose.rpy.rx = msg->data[3];
            cartesian_pose.rpy.ry = msg->data[4];
            cartesian_pose.rpy.rz = msg->data[5];
            robot.servo_p(&cartesian_pose, MoveMode::INCR); // 触发底层增量控制
        });

    auto servo_j_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/jaka_driver/servo_j_cmd", rclcpp::SensorDataQoS(),
        [&](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() < 6) return;
            JointValue joint_pose;
            for(int i=0; i<6; ++i) joint_pose.jVal[i] = msg->data[i];
            robot.servo_j(&joint_pose, MoveMode::INCR);     // 触发底层增量控制
        });

    auto stop_jog = node->create_wall_timer(chrono::seconds(3), stop_jog_callback);
    thread conn_state_thread(get_conn_scoket_state);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start");

    rclcpp::spin(node);
    
    if (conn_state_thread.joinable()) {
        conn_state_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}