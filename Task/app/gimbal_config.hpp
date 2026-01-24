#pragma once

#include "Algorithm/PID/Alg_PID.hpp"
#include "Algorithm/filter_alg/filter.hpp"
#include "Module/motor/DM/dm_driver.hpp"

namespace MM = Motor_n::MotorBaseDef_n;

#define CCM_RAM __attribute__((section(".ccmram")))

#define GIMBAL_ID 0x411
#define CHASSIS_ID 0x112
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

inline MM::Motor_Base_Config_t pitchMotorConfig =
    MM::Motor_Base_Config_t("Pitch", MM::Motor_Type_euc::DM4310)
        .SetControlSetting(
            MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
        .SetPIDConfig(
            alg_n::PidInitConfig_t // Angle PID
            {
                .Kp = 0.55f,
                .Ki = 0.01f,
                .Kd = 0.8f,
                .Kfa = 10.0f,
                .Kfb = 0.0f,
                .ActualValueSource = nullptr,
                .mode = Output_Limit | Integral_Limit | Feedforward | DerivativeFilter |
                        ChangingIntegrationRate,
                .max_out = 30.0f,
                .max_Ierror = 70.0f,
                .errorabsmax = 2.0f,
                .errorabsmin = 0.0f,
                .d_filter_num = 0.0f,
            },
            alg_n::PidInitConfig_t // Speed PID
            {
                .Kp = 0.7f,
                .Ki = 0.0f,
                .Kd = 0.3f,
                .Kfa = 1.35f,
                .ActualValueSource = nullptr,
                .mode = Output_Limit | DerivativeFilter,
                .max_out = 7.0f, // 7nm
            },
            alg_n::PidInitConfig_t // current PID
            {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .ActualValueSource = nullptr,
                .mode = Output_Limit,
                .max_out = 7.0f, // 7nm
            })

        .SetCANConfig(BSP_n::CanInitConfig_s{.can_handle = &hcan1,
                                             .tx_id = 0xE1, // Slave ID
                                             .rx_id = 0xE2, // Master ID
                                             .SAND_IDE = CAN_ID_STD})

        .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{.zero_offset = 0.0f,
                                                                   .radius = 0.05f,
                                                                   .ecd2length = 0.0f,
                                                                   .ratio = 10.0f})

        .SetOutputLimit(7.0f, -7.0f);
inline Motor_n::DmMotor_n::DmDriver_c::DM_ModePrame_s pitchMotorDMConfig = {
    .kp_min = 0,
    .kp_max = 500,
    .kd_min = 0,
    .kd_max = 5, // 这两项固定不能修改
    .v_min = -30,
    .v_max = 30,
    .p_min = -12.5,
    .p_max = 12.5,
    .t_min = -10,
    .t_max = 10, // 这三项必须与上位机软件参数一致
};

inline MM::Motor_Base_Config_t yawMotorConfig =
    MM::Motor_Base_Config_t("Yaw", MM::Motor_Type_euc::DM4310)
        .SetControlSetting(
            MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
        .SetPIDConfig(
            alg_n::PidInitConfig_t // Angle PID
            {

                .Kp = 0.5f,
                .Ki = 0.01f,
                .Kd = 6.0f,
                .Kfa = 1.0f,
                .Kfb = 0.0f,
                .ActualValueSource = nullptr,
                .mode = Output_Limit | Integral_Limit | Feedforward | ChangingIntegrationRate,
                .max_out = 30.0f,
                .max_Ierror = 100.0f,
                .errorabsmax = 1.7f,
                .errorabsmin = 0.0f},
            alg_n::PidInitConfig_t // Speed PID
            {
                .Kp = 0.6f,
                .Ki = 0.0f,
                .Kd = 0.09f,
                .Kfa = 1.0f,
                .ActualValueSource = nullptr,
                .mode = Output_Limit | DerivativeFilter | Integral_Limit | OutputFilter |
                        Feedforward | ChangingIntegrationRate,
                .max_out = 7.0f, // 7nm
                .max_Ierror = 100.0f,
                .errorabsmax = 0.0f,
                .errorabsmin = 0.0f,
                .d_filter_num = 1,
                .out_filter_num = 1.0f,
            },
            alg_n::PidInitConfig_t // current PID
            {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .ActualValueSource = nullptr,
                .mode = Output_Limit,
                .max_out = 7.0f, // 7nm
            })

        .SetCANConfig(BSP_n::CanInitConfig_s{.can_handle = &hcan2,
                                             .tx_id = 0xF1, // Slave ID
                                             .rx_id = 0xF2, // Master ID
                                             .SAND_IDE = CAN_ID_STD})

        .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{.zero_offset = 0.0f,
                                                                   .radius = 0.05f,
                                                                   .ecd2length = 0.0f,
                                                                   .ratio = 10.0f})

        .SetOutputLimit(7.0f, -7.0f);
inline Motor_n::DmMotor_n::DmDriver_c::DM_ModePrame_s yawMotorDMConfig = {
    .kp_min = 0,
    .kp_max = 500,
    .kd_min = 0,
    .kd_max = 5, // 这两项固定不能修改
    .v_min = -30,
    .v_max = 30,
    .p_min = -12.56637,
    .p_max = 12.56637,
    .t_min = -10,
    .t_max = 10, // 这三项必须与上位机软件参数一致
};

inline alg_n::PidInitConfig_t pitch_visual_angle_config // Angle PID
    {
        .Kp = 0.6f,
        .Ki = 0.0097f,
        .Kd = 0.3f,
        .Kfa = 0.0f,
        .Kfb = 0.0f,
        .ActualValueSource = nullptr,
        .mode = Output_Limit | Integral_Limit | Feedforward | DerivativeFilter |
                ChangingIntegrationRate,
        .max_out = 30.0f,
        .max_Ierror = 100.0f,
        .errorabsmax = 1.2f,
        .errorabsmin = 0.3f,
        .d_filter_num = 0.2f,
    };
inline alg_n::PidInitConfig_t pitch_visual_speed_config // Speed PID
    {
        .Kp = 0.5f,
        .Ki = 0.0f,
        .Kd = 0.2f,
        .ActualValueSource = nullptr,
        .mode = Output_Limit | DerivativeFilter,
        .max_out = 7.0f, // 7nm
    };

inline alg_n::PidInitConfig_t yaw_visual_angle_config // Angle PID
    {

        .Kp = 0.6f,
        .Ki = 0.01f,
        .Kd = 4.0f,
        .Kfa = 0.0f,
        .Kfb = 0.0f,
        .ActualValueSource = nullptr,
        .mode = Output_Limit | Integral_Limit | Feedforward | ChangingIntegrationRate,
        .max_out = 30.0f,
        .max_Ierror = 100.0f,
        .errorabsmax = 1.8f,
        .errorabsmin = 0.5f};
inline alg_n::PidInitConfig_t yaw_visual_speed_config // Speed PID
    {
        .Kp = 0.6f,
        .Ki = 0.0f,
        .Kd = 0.02f,
        .Kfa = 0.0f,
        .ActualValueSource = nullptr,
        .mode = Output_Limit | DerivativeFilter | Integral_Limit | OutputFilter | Feedforward |
                ChangingIntegrationRate,
        .max_out = 7.0f, // 7nm
        .max_Ierror = 100.0f,
        .errorabsmax = 0.0f,
        .errorabsmin = 0.0f,
        .d_filter_num = 1,
        .out_filter_num = 1.0f,
    };

typedef enum {
    GIMBAL_MANUAL,     // 手动状态
    GIMBAL_AUTOATTACK, // 自瞄状态
    GIMBAL_ZERO_FORCE, // 无力状态
    GIMBAL_FIRE_TEST   // 部署模式，底盘无力，锁编码器
} Behaviour_e;

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
    alg_n::PID_c Visual_angle_pid;
    alg_n::PID_c Visual_speed_pid;
    float filter_num = 1.0f;
    alg_n::FirstOrderFilter_c* gyro_filter; // 角速度滤波结构体
    /*控制器中间量*/
    float pos_out = 0.0f;
    float spd_out = 0.0f;
    float final_out = 0.0f;

    inline void Init()
    {
        // 初始化电机
        this->motor_ptr = new Motor_n::DmMotor_n::DmDriver_c(pitchMotorConfig, pitchMotorDMConfig);
        memset(&motor_data, 0, sizeof(motor_data));
        this->motor_ptr->Enable();
        /*控制器初始化*/
        Position_pid.Init(pitchMotorConfig.angle_PID);
        Speed_pid.Init(pitchMotorConfig.speed_PID);
        Visual_angle_pid.Init(pitch_visual_angle_config);
        Visual_speed_pid.Init(pitch_visual_speed_config);
        /*滤波器初始化*/
        gyro_filter = new alg_n::FirstOrderFilter_c(filter_num);
    }
};

/*yaw电机子类*/
class yaw_c
{
public:
    Motor_n::DmMotor_n::DmDriver_c* motor_ptr = nullptr;
    Motot_Data_t motor_data;
    alg_n::PID_c Position_pid;
    alg_n::PID_c Speed_pid;
    alg_n::PID_c Visual_angle_pid;
    alg_n::PID_c Visual_speed_pid;
    float filter_num = 1.0f;
    alg_n::FirstOrderFilter_c* gyro_filter; // 角速度滤波结构体
    /*控制器中间量*/
    float yaw_close = 0.0f; // 最短路径
    float pos_out = 0.0f;
    float spd_out = 0.0f;
    float final_out = 0.0f;

    inline void Init()
    {
        // 初始化电机
        this->motor_ptr = new Motor_n::DmMotor_n::DmDriver_c(yawMotorConfig, yawMotorDMConfig);
        memset(&motor_data, 0, sizeof(motor_data));
        this->motor_ptr->Enable();
        /*控制器初始化*/
        Position_pid.Init(yawMotorConfig.angle_PID);
        Speed_pid.Init(yawMotorConfig.speed_PID);
        Visual_angle_pid.Init(yaw_visual_angle_config);
        Visual_speed_pid.Init(yaw_visual_speed_config);
        /*滤波器初始化*/
        gyro_filter = new alg_n::FirstOrderFilter_c(filter_num);
    }
};
