#include <iostream>
#include <thread>
#include <signal.h>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <iomanip>

#include "SerialCom.h"
#include "VoyCmd.h"
#include "Kinematics.h"
#include "GridMap.h"


SerialCom* g_serial = nullptr;
CVoyCmd* g_voyCmd = nullptr;
Kinematics kinematics;

// note:设定目标速度和角速度
float target_linear_speed = 0.2; // 目标线速度，单位：米/秒
float target_angular_speed = 0;  // 目标角速度，单位：弧度/秒
float out_left_speed = 0.0f;  // 输出左轮速度，单位：弧度/秒
float out_right_speed = 0.0f; // 输出右轮速度，单位：

float wheel_distance = 0.46;      // 轮距，单位：米
float wheel_radius = 0.21 * 0.5; // 轮半径，单位：米

bool is_show_map = false; // 是否显示地图（终端和ROOT）

int main(){
    // 创建串口通信对象
    SerialCom serial;
    g_serial = &serial;
    
    // 创建机器人命令对象
    CVoyCmd voyCmd;
    g_voyCmd = &voyCmd;

    // 连接硬件
    voyCmd.m_pPhy = &serial;
    serial.SetCmd(&voyCmd);
    
    // 配置串口
    serial.SetComProp(B19200, 8, 1, 0); // 19200波特率，8N1

    std::cout << "=====================================" << std::endl;

    // 打开串口（根据需要更改端口号）
    // 常用端口：0对应/dev/ttyUSB0，1对应/dev/ttyUSB1等
    if (!serial.Create(0)) {
        std::cerr << "打开串口失败" << std::endl;
            return 1;
    }
    
    std::cout << "\n串口通信已建立" << std::endl;
    std::cout << "机器人控制系统就绪" << std::endl;



    // 设置轮距和轮半径
    kinematics.SetWheelDistance(wheel_distance);
    kinematics.SetWheelRadius(wheel_radius);

    // 创建地图
    GridMap map(0.1f, 3.0f, 3.0f); // 分辨率0.1m，地图大小3m x 3m

    // 计算运动学逆解
    kinematics.KinematicsInverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
    
    // 将计算得到的电机速度设置到 kinematics 的 motor_param 结构体中
    kinematics.UpdateMotorSpeed(0, out_left_speed, out_right_speed);
    
    std::cout << "目标线速度: " << target_linear_speed << " m/s" << " 目标角速度: " << target_angular_speed << " rad/s" << std::endl;
    std::cout << "计算得到的左轮速度：" << kinematics.GetMotorSpeed(0) << " rpm" << std::endl;
    std::cout << "计算得到的右轮速度：" << kinematics.GetMotorSpeed(1) << " rpm" << std::endl;

    std::cout << "\n按回车键开始..." << std::endl;
    std::cin.get();

    // 键盘读取
    // 设置终端为非阻塞模式
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 关闭规范模式和回显
    newt.c_cc[VMIN] = 0;  // 非阻塞读取
    newt.c_cc[VTIME] = 0; // 无超时
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "机器人运动中...按任意键停止（10秒后自动退出）" << std::endl;
    
    // 主循环
    bool IsRunning = true;
    auto loop_start_time = std::chrono::system_clock::now(); // 记录循环开始时间
    while(IsRunning){
        
        // 开始时间
        auto current_time = std::chrono::system_clock::now();
        
        // 定时退出
        // note:设定自动退出时间
        std::chrono::duration<float> elapsed = current_time - loop_start_time;
        if (elapsed.count() >= 10.0f) {
            IsRunning = false;
            std::cout << "已运行10秒，自动停止运动..." << std::endl;
        }
            
        // 手动退出
        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            IsRunning = false;
            std::cout << "检测到按键，停止运动..." << std::endl;
        }
        
        // 设置电机速度
        voyCmd.SetBothMotorsSpeed(static_cast<int>(kinematics.GetMotorSpeed(0)), static_cast<int>(kinematics.GetMotorSpeed(1)));
        
        // 添加延时，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
        // 计算循环耗时
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<float> duration_Second = end - current_time;
        auto dt = duration_Second.count(); // 循环耗时，单位：秒

        // 更新速度和里程计
        kinematics.UpdateMotorSpeed(dt, out_left_speed, out_right_speed);
        
        std::cout << std::fixed << std::setprecision(3); // 设置输出格式为固定小数点，保留3位小数
        // std::cout << "里程计信息 - 线速度: " << kinematics.GetOdem().linear_speed << " m/s, 角速度: " << kinematics.GetOdem().angular_speed << " rad/s" << std::endl;
        // std::cout << "里程计信息 - 位置: (" << kinematics.GetOdem().x << ", " << kinematics.GetOdem().y << "), 角度: " << kinematics.GetOdem().angle << " rad" << std::endl;

        // 机器人全局坐标与超声数据
        float robot_x = kinematics.GetOdem().x;    // 机器人坐标
        float robot_y = kinematics.GetOdem().y;
        float robot_yaw = kinematics.GetOdem().angle;  // 朝向0°
        float distance = 1.0f;  // 超声测到障碍物 1米

        // 3. 超声更新：波束角【固定15°】
        map.UpdateByUltrasonic(robot_x, robot_y, robot_yaw, distance, 15.0f);

    }
    if(is_show_map){
        
        // 终端打印地图
        map.PrintMap();
    
        // 使用 ROOT 可视化栅格地图
        std::cout << "\n正在生成 ROOT 栅格地图可视化..." << std::endl;
        map.PrintMapROOT("gridmap_root.png");
        std::cout << "栅格地图已保存到: /bin/gridmap_root.png" << std::endl;
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    // 停止电机
    voyCmd.SetBothMotorsSpeed(0, 0);
    std::cout << "机器人已停止" << std::endl;

    return 0;
}