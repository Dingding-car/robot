#include "Kinematics.h"

// 运动学正解
void Kinematics::KinematicsForward(float left_speed, float right_speed, float* out_linear_speed, float* out_angular_speed){

    *out_linear_speed = (left_speed + right_speed) / 2.0;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;
}

// 运动学逆解
void Kinematics::KinematicsInverse(float linear_speed, float angular_speed, float* out_left_speed, float* out_right_speed){

    *out_left_speed = linear_speed - (angular_speed * wheel_distance / 2.0);
    *out_right_speed = linear_speed + (angular_speed * wheel_distance / 2.0);
}

// 获取电机速度
int Kinematics::GetMotorSpeed(){
    // todo: 获取电机速度的实现
}

// 设置轮距
void Kinematics::SetWheelDistance(float distance){
    wheel_distance = distance;
}