#pragma once

#include "Bsp/DWT/bsp_dwt.hpp"
#include "fire_ctrl.hpp"
#include "gimbal_config.hpp"
#include "imu_task.h"
#include "robo_cmd.hpp"
#include "Vision.hpp"
#include <cstring>
#include <stdlib.h>

extern "C" {
// #include "Vision.h"
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
#pragma pack(1) // 按1字节对齐
    typedef union {
        struct {
            uint16_t shooter_heat;
            uint16_t heat_limit;
            int16_t bullet_speed;
            uint8_t robot_level : 4;
            uint8_t aim_color : 1;
            uint16_t yaw_dot : 11;
        } com_packet_data;
        uint8_t rx_data[8];
    } cancom_rx_packet;
    typedef union {
        struct {
            int16_t rc_channel2 : 11;
            int16_t rc_channel3 : 11;
            int16_t rc_dial : 11;
            uint8_t rc_s1 : 2;
            uint8_t rc_s2 : 2;
            uint8_t gimbal_pitch : 8;
            uint8_t id : 1;
            uint8_t fric_onoff : 1;
            uint8_t gimbal_mode : 3;
            uint8_t fire_mode : 1;
            uint8_t aim_mode : 2;
            uint8_t fire_speed : 4;
            uint8_t reverse : 7;
        } com_packet_data;
        uint8_t tx_data[8];
    } cancom_tx_packet;
#pragma pack()
public:
    RoboCmd_c* robo_cmd;
    BMI088Heat_c* imu;
    INS_t* ins;
    Behaviour_e gimbal_mode;
    BSP_n::Can_c* broad_com;
    cancom_rx_packet chassis_data;
    cancom_tx_packet gimbal_data;
    /*视觉相关数据*/
    Visual_Tx_t visual_tx_data;
    Visual_Rx_t* visual_data;
    float pitch_dif_target = 0.5f;
    float yaw_dif_target = 0.5f;
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

    uint64_t task_cnt;
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
    void Decode_Chassis_Data(BSP_n::Can_c* instance);
    void Gimbal2Chassis();
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
