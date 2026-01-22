#pragma once

#include "Bsp/DWT/bsp_dwt.hpp"
#include "fire_ctrl.hpp"
#include "imu_task.h"
#include "motor_config.hpp"
#include "robo_cmd.hpp"
#include <cstring>
#include <stdlib.h>

extern "C"
{
    #include "Vision.h"
}

// 运行几次发送一次
#define CONTROL_SEND_HZ(HZ)    \
    {                          \
        static int16_t hz = 0; \
        hz++;                  \
        if (hz < HZ) return;   \
        hz = 0;                \
    }
class fire_c;
/*云台类*/
class Gimbal
{
    typedef enum {
        GIMBAL_MANUAL,     // 手动状态
        GIMBAL_AUTOATTACK, // 自瞄状态
        GIMBAL_ZERO_FORCE, // 无力状态
        GIMBAL_FIRE_TEST   // 部署模式，底盘无力，锁编码器
    } Behaviour_e;

public:
    RoboCmd_c* robo_cmd;
    BMI088Heat_c* imu;
    INS_t* ins;
    Behaviour_e gimbal_mode;
    Visual_Tx_t visual_tx_data;
    Visual_Rx_t *visual_data;
    /*云台电机相关类*/
    pitch_c pitch;
    yaw_c yaw;
    /*上下限*/
    float reduce_angle = 0.0f;
    float increase_angle = 0.0f;
    float imu_max = 0.0f;
    float imu_min = 0.0f;
    /*电机调试使用*/
    Motor_n::MotorBaseDef_n::MotorBase_c* pitch_debug;
    Motor_n::MotorBaseDef_n::MotorBase_c* yaw_debug;
    /*计算相关的中间量*/
    BSP_n::DWT_c* dwt;
    uint64_t last_time;
    uint64_t dt;

    /*发射机构*/
    fire_c fire;

public:
    // 用户函数
    bool Init();
    void Loop();
    static Gimbal* GetInstance()
    {
        static Gimbal gimbal_instance; // 局部静态变量，延迟初始化
        return &gimbal_instance;
    }

private:
    void update_limit();
    void update_feedback();
    void target_set();
    void mode_set();
    void pid_init();
    void crtl_calc();
    void fire_ctrl();

    inline void enable_motor()
    {
        CONTROL_SEND_HZ(100);
        pitch.motor_ptr->Enable();
        yaw.motor_ptr->Enable();
    }

private:
    Gimbal() = default;
    user_maths_c gimbal_maths;
    // // 禁止拷贝构造和拷贝赋值
    // Gimbal(const Gimbal&) = delete;
    // Gimbal& operator=(const Gimbal&) = delete;
};
