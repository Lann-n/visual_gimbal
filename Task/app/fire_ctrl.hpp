#pragma once

#include "Algorithm/filter_alg/filter.hpp"
#include "KeyboardUtil.hpp"
#include "Module/motor/DJI/dji_driver.hpp"
#include "gimbal_config.hpp"
#include "robo_cmd.hpp"
#include "Vision.hpp"
extern "C" {
// #include "Vision.h"
}

// m/s->rpm
#define FIRESPEED_TO_MOTORRPM 264
// rpm->m/s
#define MOTORRPM_TO_FIRESPEED 0.00329 // 0.00333333333333333

#define fire_speed(x) x* FIRESPEED_TO_MOTORRPM
// 当前模式
#define TEST 0
#define GAME 1
#define NOW_STATE GAME

#define POKER_GRID 9                                               // 拨盘格数
#define SEMI_NUM 1                                                 // 连发设置
#define POKER_RATIO (5 / 2.f * 36.f)                               // 拨盘减速比
#define AN_BULLET (8192 * POKER_RATIO * (1 / ((float)POKER_GRID))) // 单个子弹电机位置增加值

/******************************************射频转换************************************************/
// 发/s——>rpm
#define FIRERATE_TO_MOTORRPM 1.f / (float)POKER_GRID * POKER_RATIO * 60
// rpm——>发/s
#define MOTORRPM_TO_FIRERATE 1.f / (FIRERATE_TO_MOTORRPM)

#define fire_rate(x) x* FIRERATE_TO_MOTORRPM

/**************************************************************************************************/

/****************************************控制参数设定*********************************************/
#define PLUCK_MAX 10000
#define FIRE_MAX 16000

// 拨蛋单发位置速度环
#define FIRE_PLUCK_P_KP 0.4 // 1
#define FIRE_PLUCK_P_KI 0   // 0
#define FIRE_PLUCK_P_KD 0   // 0
#define FIRE_PLUCK_P_MAX 4500.f

#define FIRE_PLUCK_S_KP 5
#define FIRE_PLUCK_S_KI 0.5f
#define FIRE_PLUCK_S_KD 0.1f
#define FIRE_PLUCK_S_MAX 7000.0f
#define FIRE_PLUCK_S_IMAX 3000.0f

// 拨弹连发速度环
#define FIRE_FIRE_KP 10
#define FIRE_FIRE_KI 0
#define FIRE_FIRE_KD 0

#define FIRE_LEFT_KP 25 // 27//100
#define FIRE_LEFT_KI 0
#define FIRE_LEFT_KD 0 // 90//80

#define FIRE_RIGHT_KP 25
#define FIRE_RIGHT_KI 0
#define FIRE_RIGHT_KD 0

/**************************************************************************************************/

class fire_c
{
public:
    /*发射机构状态机*/
    typedef enum {
        NO_FIRE, // 不发射
        READY,   // 准备发射
        STUCK,   // 堵转
        SEMI,    // 单发
        AUTO,    // 连发
    } mode;

    struct {
        uint16_t shoot_barrel_heat_limit;   // 当前热量限制
        uint16_t shoot_barrel_heat_current; // 当前热量
        float shoot_bullet_speed;           // 当前射击初速度
        uint8_t robot_level;                // 机器人等级
        uint8_t aim_color;
    } shoot_msg;

    Motor_n::DjiMotor_n::DjiDriver_c* left_motor;
    Motor_n::DjiMotor_n::DjiDriver_c* right_motor;
    Motor_n::DjiMotor_n::DjiDriver_c* pluck_motor;

    Motor_n::MotorBaseDef_n::MotorBase_c* left_motor_data;
    Motor_n::MotorBaseDef_n::MotorBase_c* right_motor_data;
    Motor_n::MotorBaseDef_n::MotorBase_c* pluck_motor_data;

    alg_n::PID_c semi_pos_pid;
    alg_n::PID_c semi_spd_pid;
    alg_n::PID_c auto_pid;

    float pos_out;   // 单发位置环输出
    float final_out; // 单发连发输出

    /*发射机构相关状态位数据等*/
    mode fire_mode = NO_FIRE;
    RoboCmd_c* robo_cmd; // 用于获取控制指令等
    Visual_Rx_t* visual_data;

    BSP_n::DWT_c* dwt;
    float fire_speed = 23.0f;
    int16_t dead_erro = 3096;
    // 电机给定值
    struct {
        float left_motor_speed_set;  // 左摩擦轮速度设定值
        float right_motor_speed_set; // 右摩擦轮速度设定值
        float pluck_motor_auto_set;  // 拨弹盘连发设定值
        float pluck_motor_semi_set;  // 拨弹盘单发设定值
    } motor_set_value;

public:
    void Init();
    void Loop(Behaviour_e gimbal_mode, bool gimbal_closed);
    inline void Ctrl()
    {
        Motor_n::DjiMotor_n::DjiMotorControl(); // 逐个算 PID
        Motor_n::DjiMotor_n::set_GiveCurrent(); // 把 PID 输出写进分组缓冲
        Motor_n::DjiMotor_n::MotorTransmit();   // 统一发送
    }
    inline void free_semi()
    {
        /*单发释放*/
        if ((fire_mode == SEMI) &&
            (abs(motor_set_value.pluck_motor_semi_set -
                 pluck_motor->motor_data_.motor_processed_data.total_ecd) < 3000) &&
            (abs(pluck_motor->motor_data_.motor_raw_data.feedback_speed) < 1300)) {
            fire_mode = READY;
            pluck_zero_force();
        }
    }

public:
    inline void enable_motor()
    {
        left_motor->Enable();
        right_motor->Enable();
        pluck_motor->Enable();
    }
    void fire_zero_force();
    void pluck_zero_force();

    void mode_set(Behaviour_e gimbal_mode, bool gimbal_closed);
    bool check_allow_fire();
    void ctrl_fire_motor();
    void all_fire_ctrl();
    void semi_ctrl();
    void auto_ctrl();
    void ready_ctrl();
    void pluck_ctrl();

private:
    void reset_pluck();

    void check_stuck();
};