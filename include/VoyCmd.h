#ifndef VOYCMD_H
#define VOYCMD_H

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>

// 包含接口头文件
#include "IBehavior.h"

// 前向声明
class SerialCom;

// 传感器常量
#define ULTRASONICAMOUNT 24      // 超声波传感器数量
#define INFRAREDCHAR 3           // 红外传感器字节数
#define INFRAREDMOUNT 24         // 红外传感器数量
#define MAX_BUF 1024             // 接受缓冲长度

// 机器人状态
#define STOP 0                   // 停止状态
#define FORWARD 1                // 前进状态
#define BACKWARD 2               // 后退状态
#define RIGHT 3                  // 右转状态
#define LEFT 4                   // 左转状态
#define RIGHTFRONT 5             // 右前状态
#define LEFTFRONT 6              // 左前状态
#define RIGHTBACK 7              // 右后状态
#define LEFTBACK 8               // 左后状态

class CVoyCmd {
public:
    CVoyCmd();                  // 构造函数
    virtual ~CVoyCmd();         // 析构函数

    // 机器人控制方法
    void Brake(UCHAR breakmode);                                    // 刹车
    void SetBothMotorsSpeed(int leftSpeed, int rightSpeed);         // 设置双电机速度
    void SetLMotorSpeed(int leftSpeed);                             // 设置左电机速度
    void SetRMotorSpeed(int rightSpeed);                            // 设置右电机速度
    
    // 传感器查询方法
    void QueryUltrasonicSensor();                                   // 查询超声波传感器
    void QueryInfrared();                                           // 查询红外传感器
    void QueryCompass();                                            // 查询罗盘
    
    // 自动查询方法
    void AutoQueryUSonic(UINT timeGap);                             // 自动查询超声波
    void AutoQueryInfraRed(UINT timeGap);                           // 自动查询红外
    void AutoQueryCompass(UINT timeGap);                            // 自动查询罗盘
    
    // 高级控制方法
    void SpeedByGyro(int speed);                                    // 陀螺仪直行
    void CircleByGyro(int angle, int speed);                        // 陀螺仪转向
    void Kick();                                                    // 踢球动作
    void Demarcate();                                               // 陀螺仪标定
    
    // 状态管理
    UINT GetState() const { return nState; }                        // 获取当前状态
    void SetBehavior(IBehavior* pBeh);                              // 设置行为对象
    void Parse(void* buf, int length);                              // 解析接收数据
    
    // 硬件接口
    SerialCom* m_pPhy;                                              // 物理层接口
    IBehavior* m_pBeh;                                              // 行为接口
    
    // 传感器数据
    UCHAR* m_charUltrasonic;                                        // 超声波原始数据
    BOOL* EnableUSonic;                                             // 超声波使能标志
    DOUBLE* ValUSonic;                                              // 超声波距离值
    UCHAR* m_charInfrared;                                          // 红外原始数据
    BOOL* EnableInfrared;                                           // 红外使能标志
    BOOL* ValInfrared;                                              // 红外检测值
    FLOAT m_angle;                                                  // 罗盘角度
    
    // 控制参数
    int m_iLspeed;                                                  // 左电机速度
    int m_iRspeed;                                                  // 右电机速度
    BOOL bToEndThreads;                                             // 线程结束标志
    
    // 查询间隔
    UINT QueryUSonicTime;                                           // 超声波查询间隔
    UINT QueryInfraRedTime;                                         // 红外查询间隔
    UINT QueryCompassTime;                                          // 罗盘查询间隔

    // Windows兼容的成员变量
    BOOL m_bFrameStart;                                             // 指令解析首字符标记
    UCHAR m_cLast;                                                  // 检验头两个字节用到的辅助变量
    unsigned int m_nFrameLength;                                    // 当前指令长度
    UINT nState;                                                    // 机器人状态标记
    UCHAR* m_pRcvBuf;                                               // 数据接收缓冲
    UCHAR* m_pSendBuf;                                              // 数据发送缓冲
    unsigned int m_nSendlength;                                     // 已发送字节计数
    unsigned int m_nRcvIndex;                                       // 已接收字节计数
    float m_fLSpdCoe;                                               // 左电机速度系数
    float m_fRSpdCoe;                                               // 右电机速度系数
    FLOAT m_XRoll;                                                  // X滚转角
    FLOAT m_YRoll;                                                  // Y滚转角
    BYTE ValServMotor[8];                                           // 伺服电机转角

private:
    // 线程函数
    void QueryUSonicThread(void* pParam);                           // 超声波查询线程
    void QueryInfraRedThread();                                     // 红外查询线程
    void QueryCompassThread();                                      // 罗盘查询线程
    
    // 辅助方法
    void m_UpdateState();                                           // 更新状态
    WORD m_CalculateSpeed(int speed);                               // 计算速度值
    void m_ParseFrame(UCHAR* buf, int length);                      // 解析帧数据
    void m_GenerateSendBuffer(UCHAR addr, UCHAR status, UCHAR length, UCHAR ctrlCode, UCHAR* data);  // 生成发送缓冲区
    void m_Response(UCHAR* recbuf, int length);                     // 响应处理
    void m_ResponseError();                                         // 错误响应
    BOOL m_ValidFrame(UCHAR* buf, int length);                      // 验证帧有效性
    void m_ParseBuffer(const UCHAR buf);                            // 解析缓冲区
    void m_ResetSendBuf();                                          // 重置发送缓冲区
    void m_ResetRcvBuf();                                           // 重置接收缓冲区
    DOUBLE m_CalDistance(UCHAR inUSChar);                           // 计算距离
    UCHAR m_CalSum(int length);                                     // 计算校验和
    
    // 线程管理
    std::thread* m_usonicThread;                                    // 超声波线程
    std::thread* m_infraredThread;                                  // 红外线程
    std::thread* m_compassThread;                                   // 罗盘线程
    std::atomic<bool> m_running;                                    // 运行标志
};

#endif // VOYCMD_H


