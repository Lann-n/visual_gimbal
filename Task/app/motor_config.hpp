#pragma once

#include "Module/motor/DM/dm_driver.hpp"
namespace MM = Motor_n::MotorBaseDef_n;
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

