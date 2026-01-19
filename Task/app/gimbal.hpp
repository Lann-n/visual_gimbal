#pragma once

#include "imu_task.h"
#include "motor_config.hpp"
#include "robo_cmd.hpp"

class Gimbal
{

public:
    RoboCmd_c* robo_cmd;
    BMI088Heat_c* imu;
    // 前向声明内嵌类
    class pitch_c;
    class yaw_c;
    Gimbal::pitch_c *pitch;
    Gimbal::yaw_c *yaw;


public:
    bool Init();
    static Gimbal* GetInstance()
    {
        static Gimbal gimbal_instance;
        return &gimbal_instance;
    }
    class pitch_c
    {
    public:
        Motor_n::DmMotor_n::DmDriver_c* motor_ptr;

    public:
        void Init();

    private:
        pitch_c() = default;
    };
    class yaw_c
    {
    public:
        Motor_n::DmMotor_n::DmDriver_c* motor_ptr;

    public:
        void Init();

    private:
        yaw_c() = default;
    };

private:
    Gimbal() = default;
    // 禁止拷贝构造和拷贝赋值
    Gimbal(const Gimbal&) = delete;
    Gimbal& operator=(const Gimbal&) = delete;
};
