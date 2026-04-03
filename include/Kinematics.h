#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

// 电机参数
typedef struct
{
    float per_pulse_distance; // 每脉冲对应的距离，单位：米
    float motor_speed;         // 电机速度，单位：rpm
    int last_encoder_count;     // 上一次的编码器计数
} motor_param_t;

// 里程计数据
typedef struct{
    float x;
    float y;
    float angle;
    float linear_speed;
    float angular_speed;
} odem_t;

class Kinematics
{
private:
    float wheel_distance = 0.46;       // 轮距，单位：米
    float wheel_radius = 0.21 * 0.5;   // 轮半径，单位：米
    float reduction_ratio = 15.0;       // 齿轮减速比
    motor_param_t motor_param[2]; // 电机参数数组，0为左轮，1为右轮
    odem_t odem; // 里程计数据结构
public:
    Kinematics() = default;
    ~Kinematics() = default;
    // 运动学正解
    void KinematicsForward(float left_rotation_speed, float right_rotation_speed, float* out_linear_speed, float* out_angular_speed);
    // 运动学逆解
    void KinematicsInverse(float linear_speed, float angular_speed, float* out_left_rotation_speed, float* out_right_rotation_speed);

    // 更新电机速度
    void UpdateMotorSpeed(float dt,float left_motor_speed, float right_motor_speed);
    // 获取电机速度
    float GetMotorSpeed(int motor_id);
    // 设置轮距
    void SetWheelDistance(float distance);
    // 设置轮半径
    void SetWheelRadius(float radius);

    // 获取里程计信息
    odem_t &GetOdem();
    // 更新里程计信息
    void m_UpdataOdem(float dt);
    // 角度转换
    void m_TransAngleInPi(float angle, float &out_angle);
};

#endif // __KINEMATICS_H__