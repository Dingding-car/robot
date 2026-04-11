#include "RobotBehavior.h"
#include <iostream>

void RobotBehavior::SetKinematics(Kinematics* kinematics) {
    m_kinematics = kinematics;
}

void RobotBehavior::SetCmd(CVoyCmd* pCmd) {
    m_cmd = pCmd;
}

void RobotBehavior::SetShowSensor(bool enable) {
    m_showSensorData = enable;
}

void RobotBehavior::AfterUpdateUSonic(DOUBLE* distances, BOOL* enabled, UINT state) {
    
    if (!m_showSensorData) {
        // 将传感器数据保存到成员变量
        for (int i = 0; i < ULTRASONICAMOUNT; i++) {
            m_ultraSonicData.distances[i] = distances[i];
            m_ultraSonicData.enabled[i] = enabled[i];
        }
        return;
    }

    std::cout << "\n=== 超声波传感器 ===" << std::endl;
    std::cout << "传感器:" << std::endl;
    bool hasData = false;
    for (int i = 0; i < ULTRASONICAMOUNT; i++) {
        if (enabled[i]) {
            std::cout << 24 - i << "号: " << distances[i] << "m" << std::endl;
            hasData = true;
        }
    }
    if (!hasData) {
        std::cout << "未收到超声波传感器数据" << std::endl;
    }
    std::cout << "==========================" << std::endl;
}

void RobotBehavior::AfterUpdateInfrared(BOOL* data, BOOL* enabled, UINT state) {
        if (!m_showSensorData) return;
        
        std::cout << "\n=== 红外传感器 ===" << std::endl;
        std::cout << "传感器:" << std::endl;
        bool hasData = false;

        // 8个一组打印输出，编号从24号降序到1号
        for (int i = 0; i < INFRAREDMOUNT; i++) { 
            if (enabled[i]) {
                std::cout << 24 - i << "号: " << (data[i] ? "检测到障碍物" : "无障碍物") << std::endl;
                hasData = true;
            }
        }
        
        if (!hasData) {
            std::cout << "未收到红外传感器数据" << std::endl;
        }
        std::cout << "========================" << std::endl;
}

void RobotBehavior::AfterSendCommand(UCHAR* buffer, int length, UINT state) {
        // // 输出完整的帧数据
        // std::cout << "发送帧数据: ";
        // for (int i = 0; i < length; i++) {
        //     std::cout << "0x" << std::hex << (int)buffer[i] << " ";
        // }
        // std::cout << std::dec << std::endl;
}

SensorData_t& RobotBehavior::GetSensorData() {
    return m_ultraSonicData;
}

void RobotBehavior::AfterUpdateMotorParam(UINT pos, UINT speed, UINT state) {
    // 解析传入的电机参数
    const char* sign = (speed & 0x8000) ? "-" : "+"; // 判断速度符号
    int out_speed = speed & 0x7FFF; // 获取速度
    
    // 如果关联了 Kinematics 对象，可以访问完整的电机参数
    if (m_kinematics != nullptr) {
            std::cout << "[电机调试]位置: " << pos 
                      << " 速度: " << sign << out_speed 
                      << " rpm " << std::endl;
    }
}

