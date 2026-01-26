#pragma once

#include "Algorithm/PID/Alg_PID.hpp"

#define g 9.8f // m/s^2
#define PI 3.1415926535897932384626433832795f

/* 双轴云台转动惯量*/
#define J_Pitch 0.014354002f // kg/m^2
#define J_Yaw 0.01088945f    // kg/m^2

#define m_gimbal 0.05f
#define M_offset 61.8387f // 质心与坐标系偏移

inline alg_n::PidInitConfig_t ddtheta_pitch_config = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .Kfa = 0.0f,
    .Kfb = 0.0f,
    .mode = Output_Limit | ChangingIntegrationRate,
    .max_out = 0.0f,
    .max_Ierror = 0.0f,
    .deadband = 0.0f,
    .threshold_max = 0.0f,
    .threshold_min = 0.0f,
    .errorabsmax = 0,
    .errorabsmin = 0,
    .out_filter_num = 0.0f,
};

inline alg_n::PidInitConfig_t ddtheta_yaw_config = {
    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .Kfa = 0.0f,
    .Kfb = 0.0f,
    .mode = Output_Limit | ChangingIntegrationRate,
    .max_out = 0.0f,
    .max_Ierror = 0.0f,
    .deadband = 0.0f,
    .threshold_max = 0.0f,
    .threshold_min = 0.0f,
    .errorabsmax = 0,
    .errorabsmin = 0,
    .out_filter_num = 0.0f,
};

class Visual_Contrl
{
public:
    bool use_it = false;
    struct {
        float J_pitch = J_Pitch;  // pitch转动惯量
        float mg = 1.35f;       // 质量
        float m_offset = 0.0f; // 质心与坐标系偏移
        // 角加速度补偿pid
        alg_n::PID_c pitch_pid;
        float* theta;
        float* theta_dot;
        float* theta_dd;
        float* actual_theta;
        float* actual_theta_dot;
        float pitch_out;
    } pitch_data;
    struct {
        float J_yaw = J_Yaw; // yaw转动惯量
        // 角加速度补偿pid
        alg_n::PID_c yaw_pid;
        float* theta;
        float* theta_dot;
        float* theta_dd;
        float* actual_theta;
        float* actual_theta_dot;
        float yaw_out;
    } yaw_data;

public:
    void Init(float* pitch_theta,
              float* pitch_theta_dot,
              float* pitch_theta_dd,
              float* actual_pitch_theta,
              float* actual_pitch_theta_dot,
              float* yaw_theta,
              float* yaw_theta_dot,
              float* yaw_theta_dd,
              float* actual_yaw_theta,
              float* actual_yaw_theta_dot)
    {
        pitch_data.theta = pitch_theta;
        pitch_data.theta_dot = pitch_theta_dot;
        pitch_data.theta_dd = pitch_theta_dd;
        pitch_data.actual_theta = actual_pitch_theta;
        pitch_data.actual_theta_dot = actual_pitch_theta_dot;
        yaw_data.theta = yaw_theta;
        yaw_data.theta_dot = yaw_theta_dot;
        yaw_data.theta_dd = yaw_theta_dd;
        yaw_data.actual_theta = actual_yaw_theta;
        yaw_data.actual_theta_dot = actual_yaw_theta_dot;
        pitch_data.pitch_pid.Init(ddtheta_pitch_config);
        yaw_data.yaw_pid.Init(ddtheta_yaw_config);
    }

    float gravity_compensation_f(float b)
    {
        
        return pitch_data.mg * arm_cos_f32((b + 61.8387) * (PI / 180.f));
    }
    inline float pitch_calc()
    {
        pitch_out.M_out = pitch_data.pitch_pid.Calc(*pitch_data.theta, *pitch_data.theta_dot,
                                                    *pitch_data.theta_dd, *pitch_data.actual_theta,
                                                    *pitch_data.actual_theta_dot, pitch_data.J_pitch);
        pitch_out.G_out = gravity_compensation_f(*pitch_data.actual_theta);
        pitch_out.T_out = pitch_out.M_out + pitch_out.G_out;
        return pitch_out.T_out;
    }
    inline float yaw_calc()
    {
        yaw_out.M_out =
            yaw_data.yaw_pid.Calc(*yaw_data.theta, *yaw_data.theta_dot, *yaw_data.theta_dd,
                                  *yaw_data.actual_theta, *yaw_data.actual_theta_dot, yaw_data.J_yaw);
        yaw_out.T_out = yaw_out.M_out;
        return yaw_out.T_out;
    }

private:
    struct {
        float T_out;
        float M_out;
        float G_out;
        float B_out;
    } pitch_out;
    struct {
        float T_out;
        float M_out;
        float G_out;
        float B_out;
    } yaw_out;
};