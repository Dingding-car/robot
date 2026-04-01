#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

class Kinematics
{
private:
    float wheel_distance = 0.0; // 轮距，单位：米

public:
    Kinematics(/* args */) = default;
    ~Kinematics() = default;
    // 运动学正解
    void KinematicsForward(float left_speed, float right_speed, float* out_linear_speed, float* out_angular_speed);
    // 运动学逆解
    void KinematicsInverse(float linear_speed, float angular_speed, float* out_left_speed, float* out_right_speed);
    // 获取电机速度
    int GetMotorSpeed();
    // 设置轮距
    void SetWheelDistance(float distance);
};

// Kinematics::Kinematics(/* args */)
// {
// }

// Kinematics::~Kinematics()
// {
// }


#endif // __KINEMATICS_H__