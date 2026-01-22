#pragma once

#include "Algorithm/PID/Alg_PID.hpp"
#include "Algorithm/filter_alg/filter.hpp"
#include "Module/motor/DM/dm_driver.hpp"


#define CCM_RAM __attribute__((section(".ccmram")))

/*电机参数*/
/*************************上下限 ****************************/
#define PITCH_ANGLE_MAX 57.0f
#define PITCH_ANGLE_MIN 10.5f
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

/***************************************遥感灵敏度*************************************************/
#define PITCH_DPI 0.0004f
#define YAW_DPI 0.0004f

#define PITCH_DPI_MOUSE 0.0015f
#define YAW_DPI_MOUSE 0.0015f

typedef struct {
    /*imu反馈数据*/
    float actual_imu_pos;
    float actual_gyro;
    /*电机编码数据*/
    float actual_ecd_pos;
    float actual_spd;
    float actual_torgue;
} Actual_Data_t;
typedef struct {
    Actual_Data_t actual_data; // 实际值
    float mannal_set;          // 手瞄设定值(锁imu)
    float virtual_set;         // 视觉设定值
    float encode_set;          // 编码设定值
    float final_set;           // 最终决定设定值
    float output;              // 算法计算输入值
} Motot_Data_t;

/*pitch电机子类*/
class pitch_c
{
public:
    Motor_n::DmMotor_n::DmDriver_c* motor_ptr = nullptr;
    Motot_Data_t motor_data;
    alg_n::PID_c Position_pid;
    alg_n::PID_c Speed_pid;
    float filter_num = 1.0f;
    alg_n::FirstOrderFilter_c *gyro_filter; // 角速度滤波结构体
    /*控制器中间量*/
    float pos_out = 0.0f;
    float spd_out = 0.0f;
    float final_out = 0.0f;

    void Init();
};

/*yaw电机子类*/
class yaw_c
{
public:
    Motor_n::DmMotor_n::DmDriver_c* motor_ptr = nullptr;
    Motot_Data_t motor_data;
    alg_n::PID_c Position_pid;
    alg_n::PID_c Speed_pid;
    float filter_num = 1.0f;
    alg_n::FirstOrderFilter_c *gyro_filter; // 角速度滤波结构体
    /*控制器中间量*/
    float yaw_close = 0.0f; // 最短路径
    float pos_out = 0.0f;
    float spd_out = 0.0f;
    float final_out = 0.0f;

    void Init();
};
