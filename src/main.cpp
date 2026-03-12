#include "SerialCom.h"
#include "VoyCmd.h"
#include <iostream>
#include <thread>
#include <signal.h>
#include <chrono>

// 全局对象用于信号处理
SerialCom* g_serial = nullptr;
CVoyCmd* g_voyCmd = nullptr;

// 清洁关闭的信号处理器
void signalHandler(int signal) {
    std::cout << "\n接收到信号 " << signal << "，正在关闭..." << std::endl;
    if (g_voyCmd) {
        g_voyCmd->bToEndThreads = TRUE;
        g_voyCmd->AutoQueryUSonic(0);
        g_voyCmd->AutoQueryInfraRed(0);
        g_voyCmd->AutoQueryCompass(0);
    }
    if (g_serial) {
        g_serial->SetRunning(false);
    }
    exit(0);
}

// 传感器数据显示的行为实现
class RobotBehavior : public IBehavior {
private:
    CVoyCmd* m_cmd;
    bool m_showSensorData;

public:
    RobotBehavior(bool showSensorData = true) : m_cmd(nullptr), m_showSensorData(showSensorData) {}

    void SetCmd(CVoyCmd* pCmd) override {
        m_cmd = pCmd;
    }

    void AfterUpdateUSonic(DOUBLE* distances, BOOL* enabled, UINT state) override {
        if (!m_showSensorData) return;
        
        std::cout << "\n=== 超声波传感器 ===" << std::endl;
        for (int i = 0; i < ULTRASONICAMOUNT; i++) {
            if (enabled[i]) {
                std::cout << "传感器 " << i << ": " << distances[i] << "m" << std::endl;
            }
        }
        std::cout << "==========================" << std::endl;
    }

    void AfterUpdateInfrared(UCHAR* data, BOOL* enabled, UINT state) override {
        if (!m_showSensorData) return;
        
        std::cout << "\n=== 红外传感器 ===" << std::endl;
        for (int i = 0; i < INFRAREDCHAR; i++) {
            if (enabled[i]) {
                std::cout << "红外 " << i << ": " << (data[i] ? "检测到" : "未检测") << std::endl;
            }
        }
        std::cout << "========================" << std::endl;
    }

    void AfterSendCommand(UCHAR* buffer, int length, UINT state) override {
        // 可选：显示发送的命令
        // std::cout << "Command sent, length: " << length << std::endl;
    }
};

int main() {
    // 设置信号处理器
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "=====================================" << std::endl;
    
    // 创建串口通信对象
    SerialCom serial;
    g_serial = &serial;
    
    // 创建机器人命令对象
    CVoyCmd voyCmd;
    g_voyCmd = &voyCmd;
    
    // 设置传感器数据显示行为
    RobotBehavior behavior(true);
    voyCmd.SetBehavior(&behavior);
    
    // 连接硬件
    voyCmd.m_pPhy = &serial;
    
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
    
    std::cout << "\n命令：" << std::endl;
    std::cout << "  f - 前进" << std::endl;
    std::cout << "  b - 后退" << std::endl;
    std::cout << "  l - 左转" << std::endl;
    std::cout << "  r - 右转" << std::endl;
    std::cout << "  s - 停止" << std::endl;
    std::cout << "  w - 慢速前进（测试用）" << std::endl;
    std::cout << "  q - 退出" << std::endl;
    std::cout << "  t - 测试传感器（启用自动查询）" << std::endl;
    std::cout << "  x - 停止传感器（禁用自动查询）" << std::endl;
    std::cout << "  k - 踢球动作" << std::endl;
    std::cout << "  d - 标定（陀螺仪校准）" << std::endl;
    std::cout << "\n按回车键开始..." << std::endl;
    std::cin.get();
    
    char command;
    bool running = true;
    bool sensorsEnabled = false;
    
    while (running && serial.IsRunning()) {
        std::cout << "\n命令: ";
        std::cin >> command;
        
        switch (command) {
            case 'f':
            case 'F':
                std::cout << "前进中..." << std::endl;
                voyCmd.SetBothMotorsSpeed(1000, 1000); // 以速度1000前进
                break;
                
            case 'b':
            case 'B':
                std::cout << "后退中..." << std::endl;
                voyCmd.SetBothMotorsSpeed(-1000, -1000); // 以速度1000后退
                break;
                
            case 'l':
            case 'L':
                std::cout << "左转中..." << std::endl;
                voyCmd.SetBothMotorsSpeed(-500, 500); // 左转
                break;
                
            case 'r':
            case 'R':
                std::cout << "右转中..." << std::endl;
                voyCmd.SetBothMotorsSpeed(500, -500); // 右转
                break;
                
            case 'w':
            case 'W':
                std::cout << "慢速前进中..." << std::endl;
                voyCmd.SetBothMotorsSpeed(300, 300); // 测试用慢速前进
                break;
                
            case 's':
            case 'S':
                std::cout << "停止中..." << std::endl;
                voyCmd.Brake(1); // 紧急停止
                break;
                
            case 't':
            case 'T':
                if (!sensorsEnabled) {
                    std::cout << "启用传感器..." << std::endl;
                    voyCmd.AutoQueryUSonic(500);      // 每500ms查询超声波
                    // behavior.AfterUpdateUSonic()
                    voyCmd.AutoQueryInfraRed(200);    // 每200ms查询红外
                    voyCmd.AutoQueryCompass(1000);    // 每1000ms查询罗盘
                    sensorsEnabled = true;
                    std::cout << "传感器已启用。数据将自动显示。" << std::endl;
                } else {
                    std::cout << "传感器已经启用了。" << std::endl;
                }
                break;
                
            case 'x':
            case 'X':
                if (sensorsEnabled) {
                    std::cout << "禁用传感器..." << std::endl;
                    voyCmd.AutoQueryUSonic(0);
                    voyCmd.AutoQueryInfraRed(0);
                    voyCmd.AutoQueryCompass(0);
                    sensorsEnabled = false;
                    std::cout << "传感器已禁用。" << std::endl;
                } else {
                    std::cout << "传感器已经禁用了。" << std::endl;
                }
                break;
                
            case 'd':
            case 'D':
                std::cout << "标定中（陀螺仪校准）..." << std::endl;
                voyCmd.Demarcate();
                break;
                
            case 'q':
            case 'Q':
                std::cout << "退出中..." << std::endl;
                running = false;
                break;
                
            default:
                std::cout << "未知命令。使用 f/b/l/r/s/w/q/t/x/k/d" << std::endl;
                break;
        }
        
        // 小延迟以防止串口过载
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 清洁关闭
    if (sensorsEnabled) {
        voyCmd.AutoQueryUSonic(0);
        voyCmd.AutoQueryInfraRed(0);
        voyCmd.AutoQueryCompass(0);
    }
    voyCmd.bToEndThreads = TRUE;
    serial.SetRunning(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "\n程序成功终止" << std::endl;
    return 0;
}
