#pragma once

#include "imu_task.h"
#include "motor_config.hpp"
#include "robo_cmd.hpp"

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

    /*云台电机相关类*/
    pitch_c pitch;
    yaw_c yaw;

    Behaviour_e now_mode;

public:
    // 用户函数
    bool Init();
    void mode_set();

    static Gimbal* GetInstance() 
    {
        static Gimbal gimbal_instance;  // 局部静态变量，延迟初始化
        return &gimbal_instance;
    }
private:
    Gimbal() = default;
   
    // // 禁止拷贝构造和拷贝赋值
    // Gimbal(const Gimbal&) = delete;
    // Gimbal& operator=(const Gimbal&) = delete;
};
