#include "gimbal.hpp"

bool Gimbal::Init()
{
    // 等待 IMU 初始化完成（init_flag 置 1 之后再进入主循环）
    if (getImuPtr() == nullptr || getImuPtr()->init_flag == 0) return 1;
    robo_cmd = RoboCmd_c::GetInstance();
    robo_cmd->RoboCmdInit();
    imu = getImuPtr();

    pitch->Init();
    yaw->Init();
}

void Gimbal::pitch_c::Init()
{
    MM::Motor_Base_Config_t pitchMotorConfig =
        MM::Motor_Base_Config_t("pitch", MM::Motor_Type_euc::DM4310)
            .SetControlSetting(
                MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
            .SetPIDConfig(
                alg_n::PidInitConfig_t // Angle PID
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

                },
                alg_n::PidInitConfig_t // Speed PID
                {
                    .Kp = 0.5f,
                    .Ki = 0.0f,
                    .Kd = 0.2f,
                    .ActualValueSource = nullptr,
                    .mode = Output_Limit | DerivativeFilter,
                    .max_out = 7.0f, // 7nm
                })

            .SetCANConfig(BSP_n::CanInitConfig_s{.can_handle = &hcan1,
                                                 .tx_id = 0xE1, // Slave ID
                                                 .rx_id = 0xE2, // Master ID
                                                 .can_module_callback = nullptr,
                                                 .SAND_IDE = CAN_ID_STD})

            .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{.zero_offset = 0.0f,
                                                                       .radius = 0.05f,
                                                                       .ecd2length = 0.0f,
                                                                       .ratio = 10.0f})

            .SetOutputLimit(7.0f, -7.0f);
    Motor_n::DmMotor_n::DmDriver_c::DM_ModePrame_s pitchMotorDMConfig = {
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
    motor_ptr = new Motor_n::DmMotor_n::DmDriver_c(pitchMotorConfig, pitchMotorDMConfig);
}

void Gimbal::yaw_c::Init()
{
    MM::Motor_Base_Config_t yawMotorConfig =
        MM::Motor_Base_Config_t("yaw", MM::Motor_Type_euc::DM4310)
            .SetControlSetting(
                MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
            .SetPIDConfig(
                // yaw轴角度环调这里！！代码有屎反正用的是这个！！
                //@warning
                alg_n::PidInitConfig_t // Angle PID
                {

                    .Kp = 0.65f,
                    .Ki = 0.01f,
                    .Kd = 8.0f,
                    .Kfa = 0.0f,
                    .Kfb = 0.0f,
                    .ActualValueSource = nullptr,
                    .mode = Output_Limit | Integral_Limit | Feedforward | ChangingIntegrationRate,
                    .max_out = 30.0f,
                    .max_Ierror = 100.0f,
                    .errorabsmax = 1.8f,
                    .errorabsmin = 0.5f

                },
                alg_n::PidInitConfig_t // Speed PID
                {
                    .Kp = 0.72f,
                    .Ki = 0.0f,
                    .Kd = 0.015f,
                    .Kfa = 2.0f,
                    .ActualValueSource = nullptr,
                    .mode = Output_Limit | DerivativeFilter | Integral_Limit | OutputFilter |
                            Feedforward | ChangingIntegrationRate,
                    .max_out = 7.0f, // 7nm
                    .max_Ierror = 100.0f,
                    .errorabsmax = 0.0f,
                    .errorabsmin = 0.0f,
                    .d_filter_num = 1,
                    .out_filter_num = 1.0f,
                })

            .SetCANConfig(BSP_n::CanInitConfig_s{.can_handle = &hcan2,
                                                 .tx_id = 0xF1, // Slave ID
                                                 .rx_id = 0xF2, // Master ID
                                                 .can_module_callback = nullptr,
                                                 .SAND_IDE = CAN_ID_STD})

            .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{.zero_offset = 0.0f,
                                                                       .radius = 0.05f,
                                                                       .ecd2length = 0.0f,
                                                                       .ratio = 10.0f})

            .SetOutputLimit(7.0f, -7.0f);
    Motor_n::DmMotor_n::DmDriver_c::DM_ModePrame_s yawMotorDMConfig = {
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
}