#include "VoyCmd.h"
#include "SerialCom.h"
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>

// 构造函数
CVoyCmd::CVoyCmd() {
    // 初始化传感器数组
    m_charUltrasonic = new UCHAR[ULTRASONICAMOUNT];
    EnableUSonic = new BOOL[ULTRASONICAMOUNT];
    ValUSonic = new DOUBLE[ULTRASONICAMOUNT];
    
    m_charInfrared = new UCHAR[INFRAREDCHAR];
    EnableInfrared = new BOOL[INFRAREDMOUNT];
    ValInfrared = new BOOL[INFRAREDMOUNT];

    // 初始化传感器数据
    for (int i = 0; i < ULTRASONICAMOUNT; i++) {
        ValUSonic[i] = 1.0;
        EnableUSonic[i] = TRUE;
        EnableInfrared[i] = TRUE;
    }
    
    for (int i = 0; i < INFRAREDMOUNT; i++) {
        ValInfrared[i] = FALSE;
    }
    
    // 初始化缓冲区
    m_pRcvBuf = new UCHAR[1024];
    m_pSendBuf = new UCHAR[256];
    m_nRcvIndex = 0;
    m_nSendlength = 0;
    
    // 初始化状态
    nState = STOP;
    m_iLspeed = m_iRspeed = 0;
    m_angle = 0.0f;
    
    // 初始化硬件指针
    m_pPhy = nullptr;
    m_pBeh = nullptr;
    
    // 初始化查询参数
    QueryUSonicTime = 0;
    QueryInfraRedTime = 0;
    QueryCompassTime = 0;
    bToEndThreads = FALSE;
    m_running = false;
    
    // 初始化线程
    m_usonicThread = nullptr;
    m_infraredThread = nullptr;
    m_compassThread = nullptr;
    
    // 重置缓冲区
    m_ResetRcvBuf();
    m_ResetSendBuf();
}

// 析构函数
CVoyCmd::~CVoyCmd() {
    // 停止所有线程
    bToEndThreads = TRUE;
    m_running = false;
    
    // 等待线程结束
    if (m_usonicThread && m_usonicThread->joinable()) {
        m_usonicThread->join();
        delete m_usonicThread;
    }
    
    if (m_infraredThread && m_infraredThread->joinable()) {
        m_infraredThread->join();
        delete m_infraredThread;
    }
    
    if (m_compassThread && m_compassThread->joinable()) {
        m_compassThread->join();
        delete m_compassThread;
    }
    
    // 清理内存
    delete[] m_charUltrasonic;
    delete[] EnableUSonic;
    delete[] ValUSonic;
    delete[] m_charInfrared;
    delete[] EnableInfrared;
    delete[] ValInfrared;
    delete[] m_pRcvBuf;
    delete[] m_pSendBuf;
    
    // 给线程时间退出
    if (QueryUSonicTime > QueryInfraRedTime) {
        std::this_thread::sleep_for(std::chrono::milliseconds(QueryUSonicTime + 10));
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(QueryInfraRedTime + 10));
    }
}

// 线程函数
void CVoyCmd::QueryUSonicThread() {
    UINT startTime = QueryUSonicTime;
    while (startTime == QueryUSonicTime && !bToEndThreads) {
        QueryUltrasonicSensor();
        std::this_thread::sleep_for(std::chrono::milliseconds(startTime));
    }
}

void CVoyCmd::QueryInfraRedThread() {
    UINT startTime = QueryInfraRedTime;
    while (startTime == QueryInfraRedTime && !bToEndThreads) {
        QueryInfrared();
        std::this_thread::sleep_for(std::chrono::milliseconds(startTime));
    }
}

void CVoyCmd::QueryCompassThread() {
    UINT startTime = QueryCompassTime;
    while (startTime == QueryCompassTime && !bToEndThreads) {
        QueryCompass();
        std::this_thread::sleep_for(std::chrono::milliseconds(startTime));
    }
}

// 解析接收缓冲区
void CVoyCmd::Parse(void* buf, int length) {
    if (length == 0 || buf == nullptr) return;
    
    for (int i = 0; i < length; i++) {
        m_ParseBuffer(((UCHAR*)buf)[i]);
    }
}

void CVoyCmd::m_ParseBuffer(const UCHAR buf) {
    // 查找帧开始标志
    if (buf == (UCHAR)0xaa && m_cLast == (UCHAR)0x55 && !m_bFrameStart) {
        m_pRcvBuf[0] = 0x55;
        m_pRcvBuf[1] = 0xAA;
        m_bFrameStart = true;
        return;
    }
    
    if (m_bFrameStart) {
        m_cLast = 0x00;
        m_pRcvBuf[m_nRcvIndex + 2] = buf;
        if (m_nRcvIndex == 1) {
            m_nFrameLength = (buf & 0x3f) + 4;
        }
        m_nRcvIndex++;
        
        // 接收到完整帧
        if (m_nRcvIndex == m_nFrameLength) {
            m_ParseFrame(m_pRcvBuf + 2, m_nRcvIndex);
            m_ResetRcvBuf();
        }
        
        // 缓冲区溢出保护
        if (m_nRcvIndex >= 1024) {
            m_ResetRcvBuf();
        }
    } else {
        m_cLast = buf;
    }
}

void CVoyCmd::m_ParseFrame(UCHAR* buf, int length) {
    if (!m_ValidFrame(buf, length)) {
        m_ResponseError();
        return;
    }
    
    switch (buf[2]) {
        case 0x30: { // 超声波传感器响应
            for (int i = 0; i < ULTRASONICAMOUNT; i++) {
                m_charUltrasonic[i] = buf[i + 3];
                ValUSonic[i] = m_CalDistance(m_charUltrasonic[i]);
            }
            
            if (m_pBeh != nullptr) {
                m_pBeh->AfterUpdateUSonic(ValUSonic, EnableUSonic, nState);
            }
            break;
        }
        case 0x34: { // 罗盘和陀螺仪响应
            WORD CmpsData;
            CmpsData = buf[5];
            CmpsData <<= 8;
            CmpsData |= buf[6];
            m_angle = ((FLOAT)CmpsData / 180) * 3.1415926f;
            break;
        }
        case 0x36: { // 红外传感器响应
            for (int i = 0; i < INFRAREDCHAR; i++) {
                m_charInfrared[i] = buf[i + 3];
                for (int j = 7; j >= 0; j--) {
                    if ((m_charInfrared[i] & 0x01) == 0x01) {
                        ValInfrared[j + 8 * i] = false;
                    } else {
                        ValInfrared[j + 8 * i] = true;
                    }
                    m_charInfrared[i] = m_charInfrared[i] >> 1;
                }
                m_charInfrared[i] = m_charInfrared[i] >> 1;
            }
            
            if (m_pBeh != nullptr) {
                m_pBeh->AfterUpdateInfrared(m_charInfrared, EnableInfrared, nState);
            }
            break;
        }
        case 0x21:
        case 0x24:
        case 0x25:
        case 0x26:
        case 0x2c:
        case 0x2d:
        case 0x2F:
            break;
        default:
            m_ResponseError();
            break;
    }
    
    m_Response(buf, length);
}

void CVoyCmd::m_ResetRcvBuf() {
    memset(m_pRcvBuf, 0, 1024);
    m_nRcvIndex = 0;
    m_bFrameStart = false;
    m_cLast = 0x00;
    m_nFrameLength = 0;
}

void CVoyCmd::m_ResetSendBuf() {
    memset(m_pSendBuf, 0, 256);
    m_nSendlength = 0;
}

BOOL CVoyCmd::m_ValidFrame(UCHAR* buf, int length) {
    if ((buf[1] & 0x3f) != length - 4) {
        return false;
    }
    
    int sum = 0x000000ff;
    for (int i = 0; i < length - 1; i++) {
        sum += buf[i];
    }
    
    if (buf[length - 1] != (UCHAR)(sum & 0x000000ff)) {
        return false;
    }
    
    return true;
}

void CVoyCmd::m_ResponseError() {
    std::cout << "接收到无效帧" << std::endl;
}

void CVoyCmd::m_Response(UCHAR* recbuf, int length) {
    // 有效帧接收回调
    if (m_pBeh != nullptr) {
        m_pBeh->AfterSendCommand(m_pSendBuf, m_nSendlength, nState);
    }
}

UCHAR CVoyCmd::m_CalSum(int length) {
    int temp = 0;
    for (int i = 0; i < length; i++) {
        temp += m_pSendBuf[i];
    }
    return (UCHAR)(temp & 0x000000ff);
}

DOUBLE CVoyCmd::m_CalDistance(UCHAR inUSChar) {
    return (DOUBLE)inUSChar * 0.02174;
}

void CVoyCmd::m_GenerateSendBuffer(UCHAR addr, UCHAR status, UCHAR length, UCHAR ctrlcode, UCHAR* data) {
    m_pSendBuf[0] = (UCHAR)0x55;
    m_pSendBuf[1] = (UCHAR)0xaa;
    m_pSendBuf[2] = addr;
    m_pSendBuf[3] = ((status << 6) & 0xc0) | (length & 0x3f);
    m_pSendBuf[4] = ctrlcode;
    
    if (length > 0 && data != nullptr) {
        memcpy(&m_pSendBuf[5], data, length);
    }
    
    m_pSendBuf[length + 5] = m_CalSum(length + 5);
    m_nSendlength = length + 6;
    
    // 通过物理层发送
    if (m_pPhy != nullptr) {
        m_pPhy->Send(m_pSendBuf, m_nSendlength);
    }
    
    // 回调
    if (m_pBeh != nullptr) {
        m_pBeh->AfterSendCommand(m_pSendBuf, m_nSendlength, nState);
    }
}

// 机器人控制方法
void CVoyCmd::Brake(UCHAR breakmode) {
    m_GenerateSendBuffer((UCHAR)0x01, 0, 1, (UCHAR)0x21, &breakmode);
    m_iLspeed = 0;
    m_iRspeed = 0;
    nState = STOP;
}

void CVoyCmd::SetBothMotorsSpeed(int leftSpeed, int rightSpeed) {
    if (leftSpeed == m_iLspeed && rightSpeed == m_iRspeed) {
        return;
    }
    
    m_iLspeed = leftSpeed;
    m_iRspeed = rightSpeed;
    
    WORD left = m_CalculateSpeed(leftSpeed);
    WORD right = m_CalculateSpeed(rightSpeed);
    
    UCHAR bothspeed[4];
    bothspeed[0] = (UCHAR)((left >> 8) & 0x00ff);
    bothspeed[1] = (UCHAR)(left & 0x00ff);
    bothspeed[2] = (UCHAR)((right >> 8) & 0x00ff);
    bothspeed[3] = (UCHAR)(right & 0x00ff);
    
    m_GenerateSendBuffer((UCHAR)0x01, 0, 4, (UCHAR)0x26, bothspeed);
    m_UpdateState();
}

void CVoyCmd::SetLMotorSpeed(int leftSpeed) {
    if (leftSpeed == m_iLspeed) {
        return;
    }
    
    m_iLspeed = leftSpeed;
    WORD left = m_CalculateSpeed(leftSpeed);
    
    UCHAR lspeed[2];
    lspeed[1] = (UCHAR)(left & 0x00ff);
    lspeed[0] = (UCHAR)((left >> 8) & 0x00ff);
    
    m_GenerateSendBuffer((UCHAR)0x01, 0, 2, (UCHAR)0x24, lspeed);
    m_UpdateState();
}

void CVoyCmd::SetRMotorSpeed(int rightSpeed) {
    if (rightSpeed == m_iRspeed) {
        return;
    }
    
    m_iRspeed = rightSpeed;
    WORD right = m_CalculateSpeed(rightSpeed);
    
    UCHAR rspeed[2];
    rspeed[1] = (UCHAR)(right & 0x00ff);
    rspeed[0] = (UCHAR)((right >> 8) & 0x00ff);
    
    m_GenerateSendBuffer((UCHAR)0x01, 0, 2, (UCHAR)0x25, rspeed);
    m_UpdateState();
}

// 传感器查询方法
void CVoyCmd::QueryInfrared() {
    m_GenerateSendBuffer((UCHAR)0x02, 0, 0, (UCHAR)0x36, nullptr);
}

void CVoyCmd::QueryUltrasonicSensor() {
    UCHAR ultrasonicchar[3];
    for (int i = 0; i < 3; i++) {
        ultrasonicchar[i] = 0xff;
    }
    m_GenerateSendBuffer((UCHAR)0x02, 0, 3, (UCHAR)0x30, ultrasonicchar);
}

void CVoyCmd::QueryCompass() {
    m_GenerateSendBuffer(0x03, 0, 0, 0x34, nullptr);
}

// 自动查询方法
void CVoyCmd::AutoQueryUSonic(UINT timeGap) {
    if (timeGap == QueryUSonicTime) {
        return;
    }
    
    QueryUSonicTime = timeGap;
    
    if (QueryUSonicTime != 0) {
        UCHAR sw = 0x01;
        m_GenerateSendBuffer(2, 0, 1, 0x31, &sw);
        
        // 清理现有线程
        if (m_usonicThread != nullptr) {
            if (m_usonicThread->joinable()) {
                m_usonicThread->join();
            }
            delete m_usonicThread;
            m_usonicThread = nullptr;
        }
        
        // 启动新线程
        m_usonicThread = new std::thread(&CVoyCmd::QueryUSonicThread, this);
    } else {
        UCHAR sw = 0x00;
        m_GenerateSendBuffer(2, 0, 1, 0x31, &sw);
        
        // 清理线程
        if (m_usonicThread != nullptr) {
            if (m_usonicThread->joinable()) {
                m_usonicThread->join();
            }
            delete m_usonicThread;
            m_usonicThread = nullptr;
        }
    }
}

void CVoyCmd::AutoQueryInfraRed(UINT timeGap) {
    if (timeGap == QueryInfraRedTime) {
        return;
    }
    
    QueryInfraRedTime = timeGap;
    
    if (QueryInfraRedTime != 0) {
        // 清理现有线程
        if (m_infraredThread != nullptr) {
            if (m_infraredThread->joinable()) {
                m_infraredThread->join();
            }
            delete m_infraredThread;
            m_infraredThread = nullptr;
        }
        
        // 启动新线程
        m_infraredThread = new std::thread(&CVoyCmd::QueryInfraRedThread, this);
    } else {
        // 清理线程
        if (m_infraredThread != nullptr) {
            if (m_infraredThread->joinable()) {
                m_infraredThread->join();
            }
            delete m_infraredThread;
            m_infraredThread = nullptr;
        }
    }
}

void CVoyCmd::AutoQueryCompass(UINT timeGap) {
    if (timeGap == QueryCompassTime) {
        return;
    }
    
    QueryCompassTime = timeGap;
    
    if (QueryCompassTime != 0) {
        // 清理现有线程
        if (m_compassThread != nullptr) {
            if (m_compassThread->joinable()) {
                m_compassThread->join();
            }
            delete m_compassThread;
            m_compassThread = nullptr;
        }
        
        // 启动新线程
        m_compassThread = new std::thread(&CVoyCmd::QueryCompassThread, this);
    } else {
        // 清理线程
        if (m_compassThread != nullptr) {
            if (m_compassThread->joinable()) {
                m_compassThread->join();
            }
            delete m_compassThread;
            m_compassThread = nullptr;
        }
    }
}

// 高级控制方法
void CVoyCmd::SpeedByGyro(int speed) {
    WORD wspeed = m_CalculateSpeed(speed);
    UCHAR speedData[4];
    speedData[0] = 0x00;
    speedData[1] = 0x00;
    speedData[3] = (UCHAR)(wspeed & 0x00ff);
    speedData[2] = (UCHAR)((wspeed >> 8) & 0x00ff);
    
    m_GenerateSendBuffer((UCHAR)0x01, 0, 4, 0x2C, speedData);
}

void CVoyCmd::CircleByGyro(int angle, int speed) {
    if (angle < -360 || angle > 360) {
        return;
    }
    
    WORD wAngle = m_CalculateSpeed(angle * 39000 / 3445);
    WORD wSpeed = m_CalculateSpeed(speed);
    
    UCHAR toSend[4];
    toSend[1] = (UCHAR)(wAngle & 0x00ff);
    toSend[0] = (UCHAR)((wAngle >> 8) & 0x00ff);
    toSend[3] = (UCHAR)(wSpeed & 0x00ff);
    toSend[2] = (UCHAR)((wSpeed >> 8) & 0x00ff);
    
    m_GenerateSendBuffer((UCHAR)0x01, 0, 4, 0x2D, toSend);
}

void CVoyCmd::Kick() {
    m_GenerateSendBuffer(0x04, 0, 0, 0x2f, nullptr);
}

void CVoyCmd::Demarcate() {
    m_GenerateSendBuffer(0x01, 0, 0, 0x2f, nullptr);
}

WORD CVoyCmd::m_CalculateSpeed(int speed) {
    BOOL forward = (speed >= 0);
    if (!forward) {
        speed = -speed;
    }
    
    if (speed > 30000) {
        speed = 30000;
    }
    
    WORD ret = 0;
    speed &= 0x7fff;
    
    if (forward) {
        speed &= 0x7fff;
    } else {
        speed |= 0x8000;
    }
    
    ret = speed;
    return ret;
}

void CVoyCmd::m_UpdateState() {
    // 两电机同速
    if (m_iLspeed == m_iRspeed) {
        if (0 == m_iRspeed) {
            nState = STOP;
            return;
        }
        if (m_iLspeed > 0) {
            nState = FORWARD;
        } else {
            nState = BACKWARD;
        }
        return;
    }
    
    // 电机反向
    if (m_iLspeed == -m_iRspeed) {
        if (m_iLspeed > 0) {
            nState = RIGHT;
        } else {
            nState = LEFT;
        }
        return;
    }
    
    // 复杂运动模式
    if (m_iLspeed < m_iRspeed) {
        if (m_iLspeed + m_iRspeed > 0) {
            nState = LEFTFRONT;
        } else {
            nState = RIGHTBACK;
        }
    } else {
        if (m_iLspeed + m_iRspeed > 0) {
            nState = RIGHTFRONT;
        } else {
            nState = LEFTBACK;
        }
    }
}

void CVoyCmd::SetBehavior(IBehavior* pBeh) {
    m_pBeh = pBeh;
    if (m_pBeh != nullptr) {
        m_pBeh->SetCmd(this);
    }
}
