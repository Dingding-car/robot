#include <iostream>
#include <thread>
#include <signal.h>
#include <chrono>
#include "SerialCom.h"
#include "VoyCmd.h"
#include "Kinematics.h"
#include <termios.h>
#include <unistd.h>

#include <cmath>


SerialCom* g_serial = nullptr;
CVoyCmd* g_voyCmd = nullptr;
Kinematics kinematics;

float target_linear_speed = 0.1; // 目标线速度，单位：米/秒
float target_angular_speed = 1;  // 目标角速度，单位：弧度/秒
float out_left_speed = 0.0f;  // 输出左轮速度，单位：弧度/秒
float out_right_speed = 0.0f; // 输出右轮速度，单位：

float wheel_distance = 0.46;      // 轮距，单位：米
float wheel_radius = 0.21 * 0.5; // 轮半径，单位：米

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

    // 计算运动学逆解
    kinematics.KinematicsInverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
    
    // 将计算得到的电机速度设置到 kinematics 的 motor_param 结构体中
    kinematics.UpdateMotorSpeed(out_left_speed, out_right_speed);
    
    std::cout << "目标线速度: " << target_linear_speed << " m/s" << " 目标角速度: " << target_angular_speed << " rad/s" << std::endl;
    std::cout << "计算得到的左轮速度：" << kinematics.GetMotorSpeed(0) << " rpm" << std::endl;
    std::cout << "计算得到的右轮速度：" << kinematics.GetMotorSpeed(1) << " rpm" << std::endl;

    std::cout << "\n按回车键开始..." << std::endl;
    std::cin.get();

    // 设置终端为非阻塞模式
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // 关闭规范模式和回显
    newt.c_cc[VMIN] = 0;  // 非阻塞读取
    newt.c_cc[VTIME] = 0; // 无超时
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "机器人运动中...按任意键停止" << std::endl;
    
    bool running = true;
    while(running){
        // 检查是否有键盘输入
        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0) {
            running = false;
            std::cout << "检测到按键，停止运动..." << std::endl;
        }
        
        // 设置电机速度
        voyCmd.SetBothMotorsSpeed(static_cast<int>(out_left_speed), static_cast<int>(out_right_speed));
        std::cout << "正在运动... 左轮速度: " << out_left_speed << " rpm, 右轮速度: " << out_right_speed << " rpm" << std::endl;
        
        // 添加延时，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    // 停止电机
    voyCmd.SetBothMotorsSpeed(0, 0);
    std::cout << "机器人已停止" << std::endl;

    return 0;
}