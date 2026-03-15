#ifndef SERIALCOM_H
#define SERIALCOM_H

#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <signal.h>
#include <vector>

// 包含接口头文件
#include "IPhy.h"

// 前向声明
class CVoyCmd;

// Linux兼容的串口通信类
class SerialCom : public IPhy {
private:
    int m_hCom;                    // 串口句柄
    std::atomic<bool> m_bRunning;  // 运行状态标志
    std::atomic<bool> m_bSending;  // 发送状态标志
    std::thread m_receiveThread;   // 接收线程
    std::thread m_sendThread;      // 发送线程
    
    // 串口配置参数
    int m_com;         // 串口号
    int m_baudrate;    // 波特率
    int m_bytesize;    // 数据位
    int m_stopbits;    // 停止位
    int m_parity;      // 校验位
    
    // 命令缓冲区结构
    struct CMDBUF {
        unsigned char* pCmdBuf;  // 命令缓冲区指针
        unsigned int nLen;       // 命令长度
    };
    
    std::vector<CMDBUF> m_cmdlist;  // 命令队列
    CMDBUF* m_pTempCmd;             // 发送指令缓冲成员临时指针
    
    // 线程函数
    void ReceiveThread();  // 接收线程
    void SendThread();     // 发送线程
    
    // 辅助函数
    unsigned char CalculateChecksum(const unsigned char* buf, int length);  // 计算校验和
    void ParseBuffer(const unsigned char* buf, int length);                 // 解析接收缓冲区
    void ParseFrame(const unsigned char* buf, int length);                  // 解析完整帧
    bool IsValidFrame(const unsigned char* buf, int length);                // 验证帧有效性
    
    // 生成和发送命令缓冲区
    void GenerateSendBuffer(unsigned char addr, unsigned char status, 
                           unsigned char length, unsigned char ctrlCode, unsigned char* data);
    
public:
    SerialCom();   // 构造函数
    ~SerialCom();  // 析构函数
    
    void SetComProp(int baudrate, int bytesize, int stopbits, int parity);  // 设置串口属性
    BOOL Create(int inCom);                                    // 打开串口
    void Close();                                            // 关闭串口
    void SetCmd(CVoyCmd *pCmd);                              // 设置指令类对象
    void Send(const void* pBuffer, int iLength);             // 通过缓冲队列发送数据
    void ComSend(const void* pBuffer, int iLength);          // 直接发送数据
    
    // 状态管理
    void SetRunning(bool running) { m_bRunning = running; }  // 设置运行状态
    bool IsRunning() const { return m_bRunning; }            // 获取运行状态
    
    // Windows兼容的成员变量
    CVoyCmd* m_pCmd;                                         // 指令类对象指针
};

#endif // SERIALCOM_H

