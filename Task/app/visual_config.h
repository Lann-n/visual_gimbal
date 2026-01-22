#ifndef __VISUAL_CONFIG_H
#define __VISUAL_CONFIG_H

#include "fifo.h"

typedef struct
{
    uint16_t shoot_barrel_heat_limit;   // 当前热量限制
    uint16_t shoot_barrel_heat_current; // 当前热量
    float shoot_bullet_speed;           // 当前射击初速度
    uint8_t robot_level;                // 机器人等级
    uint8_t aim_color;
} shoot_msg_t;

typedef __attribute__((packed)) struct
{
    uint8_t header;
    uint8_t detect_color;
    uint8_t reset_tracker; // 打符正反转,自瞄刷新收敛
    uint8_t now_mode;      // 当前自瞄模式  1：自瞄  2：小幅 3：大幅
    float bullet_speed;    // 子弹速度
    float Roll;
    float Pitch;
    float Yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint8_t trailer;
} Visual_Tx_t;

typedef __attribute__((packed)) struct
{
    uint8_t header;
    uint8_t fire_flag;      //视觉允许发射标志位
    float pitch; //
    float pitch_omg; // rad/s 不开发0
    float pitch_gyro; // rad/s2 不开发0
    float yaw; // deg 
    float yaw_omg;
    float yaw_gyro;
    float distance;         //目标距离，若没有目标则为-1
    float radio_yaw;        //目标方向
    uint8_t attack_mode;             //1:甩头 2：定头
    uint8_t armor_nums;     //装甲板数字
    uint8_t trailer;
} Visual_Rx_t;

typedef struct
{
    uint8_t send_buff[33];
    Visual_Tx_t tx_buff;
    Visual_Rx_t rx_buff;
    fifo_s_t *usb_fifo;
} Visual_Config;

#endif
