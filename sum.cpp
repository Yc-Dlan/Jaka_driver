/******************************************************************************
 *
 * 【JAKA机械臂 & 3D Systems Touch 遥操作控制程序】
 *
 * 版本: v7.1 (中文增强注释版)
 *
 * ============================================================================
 * ==                           核心功能与改进                           ==
 * ============================================================================
 * 1. 集成三种独立的控制模式，通过按钮2循环切换：
 *    - 模式 0: 笛卡尔空间(XYZ)位置控制。主手移动映射为机械臂末端在空间中的移动。
 *    - 模式 1: 关节空间(J1-J3)控制。主手前三关节映射控制机械臂的基座、大臂、小臂。
 *    - 模式 2: 关节空间(J4-J6)控制。主手后三关节(万向节)映射控制机械臂的手腕姿态。
 * 2. 移除了原有的rx,ry,rz姿态控制，专注于提供更明确的位置和关节控制分离。
 * 3. 客户端内置移动平均滤波和死区滤波，确保输入信号平滑、无抖动。
 * 4. 控制器端启用一阶低通滤波器(LPF)，从根本上解决伺服抖动问题。
 * 5. 在终端实时刷新显示机械臂的笛卡尔空间XYZ坐标和全部六个关节的角度信息。
 * 6. 【v7.1更新】对全部代码功能、变量、逻辑块进行了非常详细的中文注释。
 * 7. 【v7.1更新】将终端显示的关节角度单位由弧度转换为度(°)，更符合人类直觉。
 * 8. 【v7.1更新】将所有终端提示信息完全中文化。
 *
 * ============================================================================
 * ==                             操作说明                             ==
 * ============================================================================
 * - 按钮 1: 按住进入伺服模式，根据当前模式控制机械臂。
 * - 按钮 2: 短按可在 "XYZ位置" -> "关节J1-J3" -> "关节J4-J6" 三种模式间循环切换。
 * - 按钮 1+2: 同时长按2秒，安全退出程序。
 *
 ******************************************************************************/

 // 预处理指令，用于在64位Windows环境下编译时，禁用一些关于不安全函数的警告。
#ifdef _WIN64
#pragma warning (disable:4996)
#endif

// --- C++标准库与系统库 ---
#include <iostream> // 用于控制台输入输出 (cout, cerr)
#include <string>   // 用于使用字符串 (std::string)
#include <vector>   // 用于使用动态数组 (std::vector)，主要用于滤波器和关节数组
#include <cstdio>   // 用于C风格的格式化输出 (printf)，方便实现终端单行刷新
#include <chrono>   // 用于高精度时间计算，获取主循环的时间间隔(dt)
#include <cmath>    // 用于数学计算 (std::abs)
#include <conio.h>  // 用于非阻塞键盘输入 (_kbhit, _getch)

// 包含Windows核心API头文件，主要用于调用Sleep函数。
#if defined(WIN32)
# include <windows.h>
#endif

// --- 核心硬件SDK头文件 ---
#include "JAKAZuRobot.h"    // JAKA机器人控制的核心API类
#include "jktypes.h"        // JAKA SDK的数据结构与枚举
#include "jkerr.h"          // JAKA SDK的错误码
#include <HD/hd.h>          // OpenHaptics核心API
#include <HDU/hduVector.h>    // OpenHaptics的三维向量工具
#include <HDU/hduError.h>     // OpenHaptics的错误处理工具


/**
 * @brief 定义一个结构体，用于统一存储从主手设备获取的所有实时数据
 * @note 这是v7.0版本新整合的结构体，包含了位置和关节两种控制模式所需的所有数据。
 *       将所有需要从回调函数传递到主循环的数据打包在一起，可以简化数据传递过程。
 */
typedef struct {
    HDboolean m_button1State;       // 按钮1状态 (true: 按下, false: 松开)
    HDboolean m_button2State;       // 按钮2状态 (true: 按下, false: 松开)
    hduVector3Dd m_devicePosition;  // 主手的笛卡尔空间位置 (x,y,z)，用于模式0
    double m_deviceJoints[6];       // 主手的6个关节角度(弧度制)，用于模式1和2
    HDErrorInfo m_error;            // OpenHaptics的错误信息
} HapticDeviceData;

// 全局变量，用于在OpenHaptics回调函数和主循环之间传递数据。
// 回调函数运行在独立的线程中，因此需要一个全局变量作为数据交换的“信箱”。
static HapticDeviceData gServoDeviceData;

/**
 * @brief OpenHaptics的异步回调函数，以最高优先级运行，持续获取主手的最新状态。
 * @param pUserData 用户自定义数据指针，此处未使用。
 * @return HD_CALLBACK_CONTINUE 表示调度器应继续调用此回调。
 */
HDCallbackCode HDCALLBACK updateDeviceCallback(void* pUserData) {
    int nButtons = 0;
    hdBeginFrame(hdGetCurrentDevice()); // 开始一个Haptic帧，锁定设备状态以便安全读取

    // 获取按钮状态
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    gServoDeviceData.m_button1State = (nButtons & HD_DEVICE_BUTTON_1) ? TRUE : FALSE;
    gServoDeviceData.m_button2State = (nButtons & HD_DEVICE_BUTTON_2) ? TRUE : FALSE;

    // 获取笛卡尔空间位置 (用于模式0)
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);

    // 分别获取主手的前三关节角和后三万向节角度，并合并
    double base_joints[3];
    double gimbal_angles[3];
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, base_joints);     // 获取基座部分的J1-J3
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles); // 获取手腕部分的J4-J6

    // 将两组数据合并成一个完整的六维关节角数组 (用于模式1和2)
    gServoDeviceData.m_deviceJoints[0] = base_joints[0];
    gServoDeviceData.m_deviceJoints[1] = base_joints[1];
    gServoDeviceData.m_deviceJoints[2] = base_joints[2];
    gServoDeviceData.m_deviceJoints[3] = gimbal_angles[0];
    gServoDeviceData.m_deviceJoints[4] = gimbal_angles[1];
    gServoDeviceData.m_deviceJoints[5] = gimbal_angles[2];

    // 检查并记录在本次回调中发生的任何错误
    gServoDeviceData.m_error = hdGetError();
    hdEndFrame(hdGetCurrentDevice()); // 结束一个Haptic帧，解锁设备
    return HD_CALLBACK_CONTINUE;
}

/**
 * @brief OpenHaptics的同步回调函数，用于在主循环中安全地拷贝全局数据。
 * @param pUserData 指向主循环中数据存储区的指针。
 * @return HD_CALLBACK_DONE 表示数据拷贝完成。
 */
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void* pUserData) {
    // memcpy是一个内存拷贝函数，可以高效地将回调函数更新的全局数据，安全地复制到主循环的局部变量中
    memcpy(pUserData, &gServoDeviceData, sizeof(HapticDeviceData));
    return HD_CALLBACK_DONE;
}

/**
 * @brief 速度映射与缩放函数 (用于模式0：位置控制)
 * @param touch_velocity 从主手计算出的速度向量。
 * @param scale 缩放因子，用于调整机械臂的移动快慢。
 * @return 映射到JAKA机械臂坐标系下的目标速度向量。
 * @note 这里的坐标系映射关系是根据实际测试确定的，可能需要根据安装方式调整。
 */
hduVector3Dd map_and_scale_velocity(const hduVector3Dd& touch_velocity, double scale) {
    hduVector3Dd jaka_velocity(0.0, 0.0, 0.0);
    jaka_velocity[0] = touch_velocity[2] * scale; // Touch Z -> JAKA X
    jaka_velocity[1] = touch_velocity[0] * scale; // Touch X -> JAKA Y
    jaka_velocity[2] = touch_velocity[1] * scale; // Touch Y -> JAKA Z
    return jaka_velocity;
}


// =================================================================
// 主函数 (程序入口)
// =================================================================
int main(int argc, char* argv[]) {
    // --- 1. 硬件与SDK初始化 ---
    HHD hHD = HD_INVALID_HANDLE; // OpenHaptics设备句柄
    HDSchedulerHandle hUpdateHandle = 0; // OpenHaptics调度器句柄
    HDErrorInfo hd_error; // OpenHaptics错误信息结构体
    JAKAZuRobot robot; // JAKA机器人控制对象
    errno_t jaka_ret; // JAKA API函数返回值

    std::cout << "--- JAKA & Touch v7.1  ---" << std::endl;
    std::cout << "--- 正在初始化节卡机械臂与主手设备... ---" << std::endl;

    // 连接JAKA机械臂并上电、使能
    std::string jaka_ip = "10.5.5.100";
    if (robot.login_in(jaka_ip.c_str()) != ERR_SUCC) { std::cerr << "错误：登录机械臂失败。" << std::endl; return -1; }
    if (robot.power_on() != ERR_SUCC) { std::cerr << "错误：机械臂上电失败。" << std::endl; robot.login_out(); return -1; }
    if (robot.enable_robot() != ERR_SUCC) { std::cerr << "错误：机械臂使能失败。" << std::endl; robot.power_off(); robot.login_out(); return -1; }

    // 初始化3D Systems Touch设备
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(hd_error = hdGetError())) {
        hduPrintError(stderr, &hd_error, "错误：初始化主手设备失败。");
        robot.disable_robot(); robot.power_off(); robot.login_out();
        return -1;
    }

    // 启动OpenHaptics调度器，开始异步高频调用updateDeviceCallback
    hUpdateHandle = hdScheduleAsynchronous(updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);
    hdStartScheduler();
    std::cout << "--- 所有设备均已初始化成功。 ---" << std::endl;

    // --- 设置伺服模式下的控制器端滤波器 ---
    // 这是解决机械臂在伺服模式下物理抖动的关键。
    // 此处设置一个关节空间一阶低通滤波器(LPF)，截止频率为0.5Hz。
    // 较低的截止频率能更好地滤除高频抖动，但会轻微增加响应延迟。
    std::cout << "[配置] 正在设置伺服运动滤波器为关节空间低通滤波(0.5 Hz)..." << std::endl;
    errno_t filter_ret = robot.servo_move_use_joint_LPF(0.5);
    if (filter_ret != ERR_SUCC) {
        std::cerr << "[警告] 设置伺服滤波器失败。错误码: " << filter_ret << std::endl;
    }
    else {
        std::cout << "[成功] 伺服滤波器设置成功。" << std::endl;
    }

    // --- 2. 主循环参数与状态变量定义 ---

    // 打印操作说明
    std::cout << "\n--- 开始进入遥操作主循环 v7.1 ---" << std::endl;
    std::cout << "操作提示: " << std::endl;
    std::cout << "  - 按住 [按钮1] 进入伺服模式进行控制。" << std::endl;
    std::cout << "  - 单击 [按钮2] 切换模式: [XYZ位置] -> [关节1-3] -> [关节4-6]。" << std::endl;
    std::cout << "  - 同时按住 [按钮1] 和 [按钮2] 两秒钟可安全退出程序。" << std::endl;
    std::cout << "参数调整: " << std::endl;
    std::cout << "  - [+] / [-]: 增/减 位置映射比例" << std::endl;
    std::cout << "  - [}] / [{]: 增/减 关节映射比例" << std::endl;

    // --- 模式与状态变量 ---
    HapticDeviceData currentHapticData; // 存储当前帧从主手获取的完整数据
    bool wasButton1Pressed = false;     // 上一帧按钮1的状态，用于检测“首次按下”的上升沿事件
    bool wasButton2Pressed = false;     // 上一帧按钮2的状态，用于检测“单击”的上升沿事件
    int controlMode = 0;                // 当前控制模式: 0=XYZ, 1=J1-3, 2=J4-6

    // --- 模式0 (位置控制) 相关变量 ---
    hduVector3Dd last_touch_pos;        // 上一帧主手的空间位置，用于计算速度
    CartesianPose jaka_target_pose;     // 机械臂的目标笛卡尔位姿，在速度积分中不断更新
    std::vector<hduVector3Dd> velocity_history; // 用于移动平均滤波的速度历史数据队列

    // --- 模式1&2 (关节控制) 相关变量 ---
    std::vector<double> robot_initial_joints(6, 0.0); // 进入伺服时，机械臂的初始关节角，作为相对运动的基准
    std::vector<double> touch_initial_joints(6, 0.0); // 进入伺服时，主手的初始关节角，作为相对运动的“零点”
    std::vector<double> robot_target_joints(6, 0.0);  // 实时计算出的机械臂目标关节角

    // --- 通用参数 ---
    auto last_time = std::chrono::high_resolution_clock::now(); // 用于计算dt，保证运动控制与帧率无关
    const double PI = 3.141592653589793; // 定义圆周率，用于弧度到角度的转换
    const int EXIT_HOLD_FRAMES = 200;   // 长按退出需要维持的帧数 (约2秒，假设100Hz)
    int exitButtonHoldCounter = 0;      // 长按退出计数器

    // --- 客户端滤波器与映射参数 ---
    const int MOVING_AVERAGE_WINDOW_SIZE = 10; // 移动平均滤波的窗口大小，越大越平滑但延迟越高
    const double DEADZONE_VELOCITY = 20.0;     // 位置控制的速度死区 (单位: mm/s)
    const double DEADZONE_JOINT_ANGLE = 0.01;  // 关节控制的角度死区 (单位: rad, 约0.57度)
    double pos_scale_factor = 1.25;      // 位置控制的比例因子，即主手移动1mm，机械臂移动1.25mm
    double joint_scale_factor = 0.5;     // 关节控制的比例因子，即主手转动1rad，机械臂转动0.5rad

    // --- 3. 主循环开始 ---
    std::cout << "\n[信息] 当前模式: XYZ位置控制" << std::endl;
    while (true) {
        // --- 3.1. 精确计算循环时间间隔(dt) ---
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_time;
        double dt = elapsed.count();
        last_time = current_time;

        // --- 3.2. 获取主手设备最新状态 ---
        hdScheduleSynchronous(copyDeviceDataCallback, &currentHapticData, HD_MIN_SCHEDULER_PRIORITY);

        // --- 3.x. 检测键盘输入以调整参数 ---
        if (_kbhit()) { // 如果有键盘按键被按下
            char key = _getch(); // 获取按键字符
            bool param_changed = true;
            switch (key) {
            case '+': pos_scale_factor += 0.2; break;
            case '-': pos_scale_factor -= 0.2; if (pos_scale_factor < 0) pos_scale_factor = 0; break;
            case ']': case '}': joint_scale_factor += 0.05; break;
            case '[': case '{': joint_scale_factor -= 0.05; if (joint_scale_factor < 0) joint_scale_factor = 0; break;
            default: param_changed = false; break;
            }
            if (param_changed) {
                // 清除当前行并打印新参数，避免与主状态显示冲突
                printf("\r[参数更新] 位置比例: %.2f | 关节比例: %.2f                \n", pos_scale_factor, joint_scale_factor);
            }
        }

        // --- 3.3. 检测模式切换 (按钮2单击) ---
        bool isButton2Pressed = currentHapticData.m_button2State;
        if (isButton2Pressed && !wasButton2Pressed) {
            controlMode = (controlMode + 1) % 3; // 在0, 1, 2之间循环切换

            // 打印新模式信息
            switch (controlMode) {
            case 0: std::cout << "\n[信息] 模式切换: XYZ位置控制" << std::endl; break;
            case 1: std::cout << "\n[信息] 模式切换: 关节1-3 控制" << std::endl; break;
            case 2: std::cout << "\n[信息] 模式切换: 关节 4-6 控制" << std::endl; break;
            }

            // **重要**: 切换模式时，需要重置初始状态以实现平滑过渡。
            // 如果当前在伺服模式下(即按钮1按住时)切换，则立即更新初始状态，防止机械臂跳动。
            if (wasButton1Pressed) {
                // 如果新模式是位置控制
                if (controlMode == 0) {
                    jaka_ret = robot.get_tcp_position(&jaka_target_pose); // 获取机械臂当前位姿作为新目标
                    memcpy(&last_touch_pos, &currentHapticData.m_devicePosition, sizeof(hduVector3Dd)); // 主手当前位置作为新“零点”
                }
                // 如果新模式是关节控制
                else {
                    JointValue current_joints_struct;
                    robot.get_joint_position(&current_joints_struct); // 获取机械臂当前关节角
                    for (int i = 0; i < 6; ++i) {
                        robot_initial_joints[i] = current_joints_struct.jVal[i]; // 作为新基准
                        touch_initial_joints[i] = currentHapticData.m_deviceJoints[i]; // 主手当前关节角作为新“零点”
                        robot_target_joints[i] = robot_initial_joints[i]; // 目标首先等于当前
                    }
                }
            }

            velocity_history.clear(); // 清空速度历史，避免旧数据影响新模式
            Sleep(200); // 延迟一小段时间，作为按键消抖，避免一次按下触发多次切换
        }
        wasButton2Pressed = isButton2Pressed;

        // --- 3.4. 检测长按退出 (按钮1+2) ---
        if (currentHapticData.m_button1State && currentHapticData.m_button2State) {
            exitButtonHoldCounter++;
        }
        else {
            exitButtonHoldCounter = 0;
        }

        if (exitButtonHoldCounter > EXIT_HOLD_FRAMES) {
            std::cout << "\n[信息] 检测到退出指令，正在终止主循环..." << std::endl;
            if (wasButton1Pressed) {
                robot.servo_move_enable(FALSE); // 如果在伺服模式下退出，先禁用伺服
            }
            break; // 退出主循环
        }

        // --- 3.5. 主控制逻辑 (按钮1按下) ---
        bool isButton1Pressed = currentHapticData.m_button1State && (exitButtonHoldCounter == 0);

        if (isButton1Pressed) {
            // --- 按钮“首次被按下”的瞬间 ---
            // 在此初始化当前控制模式所需的基准状态，即设置运动的“起始点”
            if (!wasButton1Pressed) {
                std::cout << "\n[信息] 进入伺服模式..." << std::endl;
                robot.servo_move_enable(TRUE);

                // 根据当前激活的模式，初始化对应的“起始点”
                switch (controlMode) {
                case 0: // 初始化XYZ位置控制
                    jaka_ret = robot.get_tcp_position(&jaka_target_pose);
                    memcpy(&last_touch_pos, &currentHapticData.m_devicePosition, sizeof(hduVector3Dd));
                    velocity_history.clear();
                    break;
                case 1: // 初始化关节1-3控制
                case 2: // 初始化关节4-6控制
                {
                    JointValue initial_joints_struct;
                    jaka_ret = robot.get_joint_position(&initial_joints_struct);
                    for (int i = 0; i < 6; ++i) {
                        robot_initial_joints[i] = initial_joints_struct.jVal[i];
                        touch_initial_joints[i] = currentHapticData.m_deviceJoints[i];
                        robot_target_joints[i] = robot_initial_joints[i];
                    }
                }
                break;
                }

                if (jaka_ret != ERR_SUCC) {
                    std::cerr << "[警告] 获取机械臂初始状态失败，已自动退出伺服模式。" << std::endl;
                    robot.servo_move_enable(FALSE);
                    continue; // 跳过本次循环
                }
                wasButton1Pressed = true;
            }
            // --- 按钮“持续按住”的状态 ---
            // 在此执行对应模式的实时主从映射计算与指令发送
            else {
                // 使用switch执行当前模式的控制逻辑
                switch (controlMode) {
                    // ==================== 模式0: XYZ位置控制 ====================
                case 0: {
                    // a. 计算主手速度 (位移/时间)
                    hduVector3Dd current_touch_pos = currentHapticData.m_devicePosition;
                    hduVector3Dd raw_touch_velocity(0, 0, 0);
                    if (dt > 1e-6) { // 防止除以零
                        raw_touch_velocity = (current_touch_pos - last_touch_pos) / dt;
                    }
                    last_touch_pos = current_touch_pos;

                    // b. 对原始速度进行移动平均滤波，获得平滑速度
                    velocity_history.push_back(raw_touch_velocity);
                    if (velocity_history.size() > MOVING_AVERAGE_WINDOW_SIZE) {
                        velocity_history.erase(velocity_history.begin());
                    }
                    hduVector3Dd sum_velocity(0, 0, 0);
                    for (const auto& vel : velocity_history) {
                        sum_velocity += vel;
                    }
                    hduVector3Dd averaged_velocity = sum_velocity / velocity_history.size();

                    // c. 对平滑后的速度进行死区滤波，忽略微小抖动
                    if (averaged_velocity.magnitude() > DEADZONE_VELOCITY) {
                        // d. 速度映射、缩放并积分，更新目标位置
                        hduVector3Dd jaka_target_velocity = map_and_scale_velocity(averaged_velocity, pos_scale_factor);
                        jaka_target_pose.tran.x += jaka_target_velocity[0] * dt;
                        jaka_target_pose.tran.y += jaka_target_velocity[1] * dt;
                        jaka_target_pose.tran.z += jaka_target_velocity[2] * dt;
                    }

                    // e. 发送笛卡尔空间位置指令
                    robot.servo_p(&jaka_target_pose, ABS);
                    break;
                }
                      // ==================== 模式1 & 2: 关节控制 ====================
                case 1:
                case 2: {
                    // a. 计算主手6个关节相对于其初始“零点”的位移
                    std::vector<double> touch_delta(6, 0.0);
                    for (int i = 0; i < 6; ++i) {
                        touch_delta[i] = (currentHapticData.m_deviceJoints[i] - touch_initial_joints[i]) * joint_scale_factor;
                    }

                    // b. 对计算出的关节位移应用死区滤波
                    for (int i = 0; i < 6; ++i) {
                        if (std::abs(touch_delta[i]) < DEADZONE_JOINT_ANGLE) {
                            touch_delta[i] = 0.0;
                        }
                    }

                    // c. 根据模式，将关节位移加到机械臂的初始关节角上，得到目标关节角
                    // 注意：只更新当前模式控制的关节，其他关节保持不变，这是模式分离的关键
                    if (controlMode == 1) { // 控制 J1-J3
                        robot_target_joints[0] = robot_initial_joints[0] + touch_delta[0]; // J1: 保持不变
                        robot_target_joints[1] = robot_initial_joints[1] - touch_delta[1]; // J2: 反向
                        robot_target_joints[2] = robot_initial_joints[2] - touch_delta[2]; // J3: 反向
                    }
                    else { // 控制 J4-J6
                        for (int i = 3; i < 6; ++i) robot_target_joints[i] = robot_initial_joints[i] + touch_delta[i];
                    }

                    // d. 发送关节空间位置指令
                    JointValue target_joints_struct;
                    for (int i = 0; i < 6; ++i) {
                        target_joints_struct.jVal[i] = robot_target_joints[i];
                    }
                    robot.servo_j(&target_joints_struct, ABS);
                    break;
                }
                }
            }
        }
        // --- 按钮松开的状态 ---
        else {
            if (wasButton1Pressed) {
                printf("\n"); // 打印一个换行符，避免终端输出被覆盖
                std::cout << "[信息] 退出伺服模式。" << std::endl;
                robot.servo_move_enable(FALSE);
                wasButton1Pressed = false;
            }
        }

        // --- 3.6. 终端信息实时显示 ---
        // 在伺服模式下，持续获取并打印机械臂的实际状态
        if (wasButton1Pressed) {
            CartesianPose actual_pose;
            JointValue actual_joints;
            robot.get_tcp_position(&actual_pose);
            robot.get_joint_position(&actual_joints);

            // 使用printf实现单行刷新，\r让光标回到行首
            // 将弧度(rad)转换为角度(deg)进行显示: deg = rad * 180 / PI
            printf("\rP_Scale:%.2f J_Scale:%.2f | XYZ(mm): %.1f,%.1f,%.1f | RPY(°): %.1f,%.1f,%.1f | J(°): %.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
                pos_scale_factor, joint_scale_factor,
                actual_pose.tran.x, actual_pose.tran.y, actual_pose.tran.z,
                actual_pose.rpy.rx * 180.0 / PI, actual_pose.rpy.ry * 180.0 / PI, actual_pose.rpy.rz * 180.0 / PI,
                actual_joints.jVal[0] * 180.0 / PI, actual_joints.jVal[1] * 180.0 / PI, actual_joints.jVal[2] * 180.0 / PI,
                actual_joints.jVal[3] * 180.0 / PI, actual_joints.jVal[4] * 180.0 / PI, actual_joints.jVal[5] * 180.0 / PI);
            fflush(stdout); // 强制刷新输出缓冲区，确保信息立即显示
        }

        // --- 3.7. 错误检查与循环延时 ---
        if (HD_DEVICE_ERROR(currentHapticData.m_error)) {
            hduPrintError(stderr, &hd_error, "错误：主手设备在循环中检测到错误。");
            if (hduIsSchedulerError(&currentHapticData.m_error)) {
                std::cerr << "[致命错误] 调度器错误，程序即将退出。" << std::endl;
                break;
            }
        }
        Sleep(1); // 短暂延时1ms，避免CPU占用过高，主循环频率理论可达1000Hz
    }

    // --- 4. 清理和安全退出 ---
    std::cout << "\n\n程序正在清理资源并退出..." << std::endl;
    hdStopScheduler();
    hdUnschedule(hUpdateHandle);
    hdDisableDevice(hHD);
    // 在最终应用中，如果需要让机械臂在程序结束后保持使能，可以注释掉下面的行
    // robot.disable_robot(); 
    // robot.power_off();
    // robot.login_out();
    std::cout << "所有资源已释放。再见！" << std::endl;

    return 0; // 程序正常退出
}
