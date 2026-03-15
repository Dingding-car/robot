#include "../include/SerialCom.h"
#include "../include/VoyCmd.h"
#include <iostream>

// 构造函数
SerialCom::SerialCom() : IPhy() {
    m_hCom = -1;                    // 初始化串口句柄
    m_bRunning = false;             // 初始化运行状态
    m_bSending = false;             // 初始化发送状态
    m_pCmd = nullptr;               // 初始化指令类对象指针
    
    // 默认串口设置
    m_baudrate = B19200;            // 波特率19200
    m_bytesize = 8;                 // 数据位8位
    m_stopbits = 1;                 // 停止位1位
    m_parity = 0;                   // 无校验
}

// 析构函数
SerialCom::~SerialCom() {
    m_bRunning = false;             // 停止运行
    
    // 等待接收线程结束
    if (m_receiveThread.joinable()) {
        m_receiveThread.join();
    }
    
    // 等待发送线程结束
    if (m_sendThread.joinable()) {
        m_sendThread.join();
    }
    
    // 清理命令列表
    for (auto& cmd : m_cmdlist) {
        delete[] cmd.pCmdBuf;
    }
    m_cmdlist.clear();
    
    // 关闭串口
    if (m_hCom != -1) {
        close(m_hCom);
        m_hCom = -1;
    }
}

// 设置串口属性
void SerialCom::SetComProp(int baudrate, int bytesize, int stopbits, int parity) {
    m_baudrate = baudrate;          // 设置波特率
    m_bytesize = bytesize;          // 设置数据位
    m_stopbits = stopbits;          // 设置停止位
    m_parity = parity;              // 设置校验位
}

// 打开串口
BOOL SerialCom::Create(int inCom) {
    // 检查是否已在运行
    if (m_bRunning) {
        std::cerr << "串口已在运行中" << std::endl;
        return false;
    }
    
    m_com = inCom;                    // 保存串口号
    
    // 构建设备路径
    std::string devicePath = "/dev/ttyUSB" + std::to_string(inCom);
    
    // 打开串口
    m_hCom = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_hCom == -1) {
        std::cerr << "打开串口失败: " << devicePath << std::endl;
        return false;
    }
    
    // 配置串口参数
    struct termios tty;
    if (tcgetattr(m_hCom, &tty) != 0) {
        std::cerr << "获取串口属性失败" << std::endl;
        close(m_hCom);
        m_hCom = -1;
        return false;
    }
    
    // 设置波特率
    cfsetospeed(&tty, m_baudrate);
    cfsetispeed(&tty, m_baudrate);
    
    // 设置数据位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    // 设置校验位
    if (m_parity == 1) {            // 奇校验
        tty.c_cflag |= PARENB;
        tty.c_cflag |= PARODD;
    } else if (m_parity == 2) {     // 偶校验
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD;
    } else {                        // 无校验
        tty.c_cflag &= ~PARENB;
    }
    
    // 设置停止位
    if (m_stopbits == 2) {          // 2位停止位
        tty.c_cflag |= CSTOPB;
    } else {                        // 1位停止位
        tty.c_cflag &= ~CSTOPB;
    }
    
    // 设置其他标志位
    tty.c_cflag |= (CLOCAL | CREAD);                    // 本地连接，接收使能
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);     // 非标准模式，不回显
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);             // 不使用流控制
    tty.c_oflag &= ~OPOST;                              // 原始输出模式
    
    // 设置超时
    tty.c_cc[VMIN] = 1;   // 至少接收1个字符
    tty.c_cc[VTIME] = 5;  // 超时时间0.5秒
    
    // 应用串口设置
    if (tcsetattr(m_hCom, TCSANOW, &tty) != 0) {
        std::cerr << "设置串口属性失败" << std::endl;
        close(m_hCom);
        m_hCom = -1;
        return false;
    }
    
    m_bRunning = true;              // 设置运行状态
    
    // 启动接收线程
    m_receiveThread = std::thread(&SerialCom::ReceiveThread, this);
    
    // 启动发送线程
    m_sendThread = std::thread(&SerialCom::SendThread, this);
    
    std::cout << "串口 " << devicePath << " 打开成功" << std::endl;
    return true;
}

// 关闭串口
void SerialCom::Close() {
    m_bRunning = false;             // 停止运行
    
    if (m_hCom != -1) {
        close(m_hCom);
        m_hCom = -1;
    }
}

// 设置指令类对象
void SerialCom::SetCmd(CVoyCmd *pCmd) {
    m_pCmd = pCmd;
    if (m_pCmd != nullptr) {
        m_pCmd->m_pPhy = this;
    }
}

// 通过缓冲队列发送数据
void SerialCom::Send(const void* pBuffer, int iLength) {
    if (!m_bRunning || !pBuffer || iLength <= 0) {
        return;
    }
    
    // 创建命令缓冲区
    CMDBUF cmd;
    cmd.pCmdBuf = new unsigned char[iLength];
    memcpy(cmd.pCmdBuf, pBuffer, iLength);
    cmd.nLen = iLength;
    
    // 添加到命令队列
    m_cmdlist.push_back(cmd);
}

// 直接发送数据
void SerialCom::ComSend(const void* pBuffer, int iLength) {
    if (m_hCom == -1 || !pBuffer || iLength <= 0) {
        return;
    }
    
    m_bSending = true;              // 设置发送状态
    
    // 写入数据到串口
    ssize_t bytesWritten = write(m_hCom, pBuffer, iLength);
    if (bytesWritten != iLength) {
        std::cerr << "写入串口失败" << std::endl;
    }
    
    // 短暂延时确保数据发送完成
    usleep(10000);                  // 10ms
    
    m_bSending = false;             // 清除发送状态
}

// 接收线程函数
void SerialCom::ReceiveThread() {
    unsigned char buffer[1024];
    fd_set readfds;
    struct timeval timeout;
    
    while (m_bRunning) {
        // 清空文件描述符集合
        FD_ZERO(&readfds);
        FD_SET(m_hCom, &readfds);
        
        // 设置超时时间
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;   // 100ms超时
        
        // 监控串口是否有数据可读
        int result = select(m_hCom + 1, &readfds, NULL, NULL, &timeout);
        
        if (result > 0 && FD_ISSET(m_hCom, &readfds)) {
            // 读取串口数据
            ssize_t bytesRead = read(m_hCom, buffer, sizeof(buffer));
            if (bytesRead > 0) {
                // 调用VoyCmd的Parse方法解析数据
                if (m_pCmd != nullptr) {
                    m_pCmd->Parse(buffer, bytesRead);
                }
            }
        } else if (result < 0) {
            std::cerr << "select()函数出错" << std::endl;
            break;
        }
    }
}

// 发送线程函数
void SerialCom::SendThread() {
    while (m_bRunning) {
        if (!m_cmdlist.empty()) {
            // 等待当前发送完成
            while (m_bSending && m_bRunning) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            if (!m_bRunning) break;
            
            // 发送队列中的第一个命令
            CMDBUF cmd = m_cmdlist.front();
            m_cmdlist.erase(m_cmdlist.begin());
            
            ComSend(cmd.pCmdBuf, cmd.nLen);
            
            // 清理内存
            delete[] cmd.pCmdBuf;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// 计算校验和
unsigned char SerialCom::CalculateChecksum(const unsigned char* buf, int length) {
    unsigned int sum = 0xFF;
    for (int i = 0; i < length; i++) {
        sum += buf[i];
    }
    return (unsigned char)(sum & 0xFF);
}

// 解析接收缓冲区
void SerialCom::ParseBuffer(const unsigned char* buf, int length) {
    static bool frameStart = false;     // 帧开始标志
    static unsigned char lastByte = 0;  // 上一个字节
    static unsigned char recvBuffer[1024];  // 接收缓冲区
    static int recvIndex = 0;           // 接收索引
    static int frameLength = 0;         // 帧长度
    
    for (int i = 0; i < length; i++) {
        unsigned char currentByte = buf[i];
        
        // 查找帧开始标志
        if (currentByte == 0xAA && lastByte == 0x55 && !frameStart) {
            recvBuffer[0] = 0x55;
            recvBuffer[1] = 0xAA;
            frameStart = true;
            recvIndex = 2;
            continue;
        }
        
        if (frameStart) {
            recvBuffer[recvIndex] = currentByte;
            if (recvIndex == 2) {
                frameLength = (currentByte & 0x3F) + 4;
            }
            recvIndex++;
            
            // 接收到完整帧
            if (recvIndex == (frameLength + 2)) {
                ParseFrame(recvBuffer, frameLength);
                
                // 重置为下一帧做准备
                frameStart = false;
                recvIndex = 0;
                frameLength = 0;
            }
            
            // 缓冲区溢出保护
            if (recvIndex >= 1024) {
                frameStart = false;
                recvIndex = 0;
            }
        }
        
        lastByte = currentByte;
    }
}

// 解析完整帧
void SerialCom::ParseFrame(const unsigned char* buf, int length) {
    if (!IsValidFrame(buf, length)) {
        std::cout << "接收到无效帧" << std::endl;
        return;
    }
    
    // 处理不同类型的命令
    unsigned char ctrlCode = buf[4];
    
    switch (ctrlCode) {
        case 0x30: // 超声波传感器响应
            std::cout << "超声波传感器数据接收" << std::endl;
            break;
        case 0x34: // 罗盘/陀螺仪响应
            std::cout << "罗盘/陀螺仪数据接收" << std::endl;
            break;
        case 0x36: // 红外传感器响应
            std::cout << "红外传感器数据接收" << std::endl;
            break;
        case 0x21: // 刹车响应
            std::cout << "刹车命令确认" << std::endl;
            break;
        case 0x24: // 左电机速度响应
            std::cout << "左电机速度设置" << std::endl;
            break;
        case 0x25: // 右电机速度响应
            std::cout << "右电机速度设置" << std::endl;
            break;
        case 0x26: // 双电机速度响应
            std::cout << "双电机速度设置" << std::endl;
            break;
        default:
            std::cout << "未知命令响应: 0x" << std::hex << (int)ctrlCode << std::dec << std::endl;
            break;
    }
}

// 验证帧有效性
bool SerialCom::IsValidFrame(const unsigned char* buf, int length) {
    // if (length < 6) return false;
    
    // 检查长度字段
    if ((buf[3] & 0x3F) != length - 4) {
        return false;
    }
    
    // 检查校验和
    unsigned char checksum = CalculateChecksum(buf, length - 1);
    if (buf[length - 1] != checksum) {
        return false;
    }
    
    return true;
}

// 生成和发送命令缓冲区
void SerialCom::GenerateSendBuffer(unsigned char addr, unsigned char status, 
                                  unsigned char length, unsigned char ctrlCode, unsigned char* data) {
    unsigned char sendBuffer[256];
    
    // 构建命令帧
    sendBuffer[0] = 0x55;                                    // 帧头1
    sendBuffer[1] = 0xAA;                                    // 帧头2
    sendBuffer[2] = addr;                                    // 地址
    sendBuffer[3] = ((status << 6) & 0xC0) | (length & 0x3F); // 状态和长度
    sendBuffer[4] = ctrlCode;                                // 控制码
    
    // 添加数据
    if (length > 0 && data != nullptr) {
        memcpy(&sendBuffer[5], data, length);
    }
    
    // 计算校验和
    unsigned char checksum = CalculateChecksum(sendBuffer, length + 5);
    sendBuffer[length + 5] = checksum;
    
    // 调试输出 - 打印完整命令帧
    std::cout << "发送命令帧: ";
    for (int i = 0; i < length + 6; i++) {
        std::cout << "0x" << std::hex << (int)sendBuffer[i] << " ";
    }
    std::cout << std::dec << std::endl;
    
    // 发送数据
    Send(sendBuffer, length + 6);
}
