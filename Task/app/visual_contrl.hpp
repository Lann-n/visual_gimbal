#pragma once

#include "Algorithm/PID/Alg_PID.hpp"

#define g 9.8f // m/s^2
#define PI 3.1415926535897932384626433832795f

/* 双轴云台转动惯量*/
#define J_Pitch 0.014354002f // kg/m^2
#define J_Yaw 0.01088945f    // kg/m^2

#define m_gimbal 0.05f
#define M_offset 61.8387f // 质心与坐标系偏移

class Visual_Contrl
{
public:
    float J = 0.0f; // 转动惯量
    float m = 0.0f; // 质量
    float m_offset = 0.0f;// 质心与坐标系偏移
public:
    alg_n::PID_c ddtheta_pid;

private:
    float T_out;
    float G_out;
    float B_out;
};