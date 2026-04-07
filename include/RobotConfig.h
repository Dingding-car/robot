// 补充缺失的类型/宏定义（可放到单独的头文件如 "RobotConfig.h" 中）
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// 替换自定义类型为标准类型，或明确别名
using DOUBLE = double;
using BOOL = bool;
using UINT = unsigned int;
using UCHAR = unsigned char;

// 定义传感器数量宏（根据实际需求调整值）
#define ULTRASONICAMOUNT 8   // 超声波传感器数量
#define INFRAREDMOUNT 24     // 红外传感器数量

#endif // ROBOT_CONFIG_H