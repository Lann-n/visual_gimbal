#pragma once

#include "Module/motor/DM/dm_driver.hpp"

/*电机参数*/
/*Pitch*/
/*位置环PID*/
#define PITCH_ANGLE_KP 0.6f
#define PITCH_ANGLE_KI 0.0097f
#define PITCH_ANGLE_KD 0.3f
#define PITCH_ANGLE_KFA 0.0f
#define PITCH_ANGLE_KFB 0.0f
/*速度环PID*/
#define PITCH_SPEED_KP 0.5f
#define PITCH_SPEED_KI 0.0f
#define PITCH_SPEED_KD 0.2f
#define PITCH_SPEED_KFA 0.0f
#define PITCH_SPEED_KFB 0.0f

/*Yaw*/
/*位置环PID*/
#define YAW_ANGLE_KP 0.6f
#define YAW_ANGLE_KI 0.0097f
#define YAW_ANGLE_KD 0.3f
#define YAW_ANGLE_KFA 0.0f
#define YAW_ANGLE_KFB 0.0f
/*速度环PID*/
#define YAW_SPEED_KP 0.5f
#define YAW_SPEED_KI 0.0f
#define YAW_SPEED_KD 0.2f
#define YAW_SPEED_KFA 0.0f
#define YAW_SPEED_KFB 0.0f

typedef struct {
    float actual_imu_pos;
    float actual_ecd_pos;
    float actual_speed;
    float actual_current;
    float actual_gyro;
} Actual_Data_t;
typedef struct {
    Actual_Data_t actual_data; // 实际值
    float mannal_set;          // 手瞄设定值
    float virtual_set;         // 视觉设定值
    float final_set;           // 最终决定设定值
    float output;              // 算法计算输入值
} Motot_Data_t;

/*pitch电机子类*/
class pitch_c
{
public:
    Motor_n::DmMotor_n::DmDriver_c* motor_ptr = nullptr;
    Motot_Data_t motor_data;
    void Init();
};

/*yaw电机子类*/
class yaw_c
{
public:
    Motor_n::DmMotor_n::DmDriver_c* motor_ptr = nullptr;
    Motot_Data_t motor_data;
    void Init();
};