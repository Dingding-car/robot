#include "Kinematics.h"
#include <cmath>

// 运动学正解
void Kinematics::KinematicsForward(float left_rotation_speed, float right_rotation_speed, float* out_linear_speed, float* out_angular_speed){

    float v_left = (M_PI * wheel_radius * left_rotation_speed) / (reduction_ratio * 30);
    float v_right = (M_PI * wheel_radius * right_rotation_speed) / (reduction_ratio * 30);
    *out_linear_speed = (v_left + v_right) * 0.5;
    *out_angular_speed = (v_right - v_left) / wheel_distance;
}

// 运动学逆解
void Kinematics::KinematicsInverse(float linear_speed, float angular_speed, float* out_left_rotation_speed, float* out_right_rotation_speed){

    float v_left = linear_speed - (angular_speed * wheel_distance / 2.0);
    float v_right = linear_speed + (angular_speed * wheel_distance / 2.0);
    *out_left_rotation_speed = (2*30 * reduction_ratio * v_left) / (M_PI * wheel_radius);
    *out_right_rotation_speed = (2*30 * reduction_ratio * v_right) / (M_PI * wheel_radius);

}

// 更新电机速度
void Kinematics::UpdateMotorSpeed(float dt, float left_motor_speed, float right_motor_speed){

    motor_param[0].motor_speed = left_motor_speed;
    motor_param[1].motor_speed = right_motor_speed;
    this->m_UpdataOdem(dt);
}

// 获取电机速度
float Kinematics::GetMotorSpeed(int motor_id){

    if(motor_id < 0 || motor_id >= 2){
        return -1; // 无效的电机ID
    }
    return motor_param[motor_id].motor_speed;
}

// 设置轮距
void Kinematics::SetWheelDistance(float distance){
    wheel_distance = distance;
}

// 设置轮半径
void Kinematics::SetWheelRadius(float radius){
    wheel_radius = radius;
}

// 获取里程计信息
odem_t& Kinematics::GetOdem(){
    return odem;
}

// 角度转换
void Kinematics::m_TransAngleInPi(float angle, float &out_angle){
    out_angle = fmod(angle + M_PI, M_PI * 2);
    out_angle -= M_PI;
}

// 更新里程计信息
void Kinematics::m_UpdataOdem(float dt) {
    // todo: 更新里程计信息的实现
    float dt_s = dt; // 单位：秒
    this->KinematicsForward(motor_param[0].motor_speed, motor_param[1].motor_speed, &odem.linear_speed, &odem.angular_speed);

    // 角度积分
    odem.angle += odem.angular_speed * dt_s;
    m_TransAngleInPi(odem.angle, odem.angle);
    
    // 局部坐标系下前进距离
    float delta_distance = odem.linear_speed * dt_s;

    // 更新全局坐标系下的位置
    odem.x += delta_distance * std::cos(odem.angle);
    odem.y += delta_distance * std::sin(odem.angle);
};