#include "gimbal.hpp"

namespace MM = Motor_n::MotorBaseDef_n;

bool Gimbal::Init()
{
    // 等待 IMU 初始化完成（init_flag 置 1 之后再进入主循环）
    if (getImuPtr() == nullptr || getImuPtr()->init_flag == 0) return 1;
    gimbal_mode = GIMBAL_ZERO_FORCE;
    robo_cmd = RoboCmd_c::GetInstance();
    robo_cmd->RoboCmdInit();
    imu = getImuPtr();
    ins = &imu->INS;

    pitch.Init();
    yaw.Init();
    pitch_debug = &pitch.motor_ptr->get_base();
    yaw_debug = &yaw.motor_ptr->get_base();
    fire.Init();

    dwt = BSP_n::DWT_c::Get_DwtInstance();
    Virtual_Init();
    visual_data = Get_virtual_recive_ptr();
    return 0;
}

void Gimbal::Loop()
{
    Virtual_recive(); // 获取视觉数据
    if (gimbal_mode == GIMBAL_ZERO_FORCE) {
        fire.fire_zero_force();
        fire.pluck_zero_force();
    } else {
        fire.Loop();
        if (gimbal_mode == GIMBAL_MANUAL) {
            // fire.all_fire_ctrl();
        } else if (gimbal_mode == GIMBAL_AUTOATTACK) {
        }
    }
    fire.pluck_ctrl();
    Motor_n::DjiMotor_n::DjiMotorControl(); // 逐个算 PID
    Motor_n::DjiMotor_n::set_GiveCurrent(); // 把 PID 输出写进分组缓冲
    Motor_n::DjiMotor_n::MotorTransmit();   // 统一发送

    update_feedback();
    mode_set();
    last_time = dwt->GetTimeline_us();
    crtl_calc();
    dt = dwt->GetTimeline_us() - last_time;
    pitch.motor_ptr->Transmit(1);
    yaw.motor_ptr->Transmit(1);
}

void pitch_c::Init()
{
    MM::Motor_Base_Config_t pitchMotorConfig =
        MM::Motor_Base_Config_t("Pitch", MM::Motor_Type_euc::DM4310)
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
    // 初始化电机
    this->motor_ptr = new Motor_n::DmMotor_n::DmDriver_c(pitchMotorConfig, pitchMotorDMConfig);
    memset(&motor_data, 0, sizeof(motor_data));
    this->motor_ptr->Enable();
    /*控制器初始化*/
    Position_pid.Init(pitchMotorConfig.angle_PID);
    Position_pid.Clear();
    Speed_pid.Init(pitchMotorConfig.speed_PID);
    Speed_pid.Clear();
    /*滤波器初始化*/
    gyro_filter = new alg_n::FirstOrderFilter_c(filter_num);
}

void yaw_c::Init()
{
    MM::Motor_Base_Config_t yawMotorConfig =
        MM::Motor_Base_Config_t("Yaw", MM::Motor_Type_euc::DM4310)
            .SetControlSetting(
                MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::ANGLE_AND_SPEED_LOOP})
            .SetPIDConfig(
                // yaw轴角度环调这里！！代码有屎反正用的是这个！！
                //@warning
                alg_n::PidInitConfig_t // Angle PID
                {

                    .Kp = 0.5f,
                    .Ki = 0.015f,
                    .Kd = 1.0f,
                    .Kfa = 1.2f,
                    .Kfb = 0.0f,
                    .ActualValueSource = nullptr,
                    .mode = Output_Limit | Integral_Limit | Feedforward | ChangingIntegrationRate,
                    .max_out = 30.0f,
                    .max_Ierror = 30.0f,
                    .errorabsmax = 1.15f,
                    .errorabsmin = 0.85f},
                alg_n::PidInitConfig_t // Speed PID
                {
                    .Kp = 0.72f,
                    .Ki = 0.0f,
                    .Kd = 0.08f,
                    .Kfa = 1.0f,
                    .ActualValueSource = nullptr,
                    .mode = Output_Limit | DerivativeFilter | Integral_Limit | Feedforward |
                            ChangingIntegrationRate,
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
    // 初始化电机
    this->motor_ptr = new Motor_n::DmMotor_n::DmDriver_c(yawMotorConfig, yawMotorDMConfig);
    memset(&motor_data, 0, sizeof(motor_data));
    this->motor_ptr->Enable();
    /*控制器初始化*/
    Position_pid.Init(yawMotorConfig.angle_PID);
    Position_pid.Clear();
    Speed_pid.Init(yawMotorConfig.speed_PID);
    Speed_pid.Clear();
    /*滤波器初始化*/
    gyro_filter = new alg_n::FirstOrderFilter_c(filter_num);
}

/**
 * @brief 基于pitch编码值更新imu上下限
 *
 */
void Gimbal::update_limit()
{
    reduce_angle = (PITCH_ANGLE_MIN + 2.0f) - pitch.motor_data.actual_data.actual_ecd_pos;
    increase_angle = (PITCH_ANGLE_MAX - 2.0f) - pitch.motor_data.actual_data.actual_ecd_pos;
    user_value_limit(reduce_angle, (PITCH_ANGLE_MIN - PITCH_ANGLE_MAX),
                     (PITCH_ANGLE_MAX - PITCH_ANGLE_MIN));
    user_value_limit(increase_angle, (PITCH_ANGLE_MIN - PITCH_ANGLE_MAX),
                     (PITCH_ANGLE_MAX - PITCH_ANGLE_MIN));
    imu_max = pitch.motor_data.actual_data.actual_imu_pos + increase_angle;
    imu_min = pitch.motor_data.actual_data.actual_imu_pos + reduce_angle;
}

void Gimbal::update_feedback()
{
    /*电机使能,防止数据不更新*/
    enable_motor();

    /*pitch数据更新*/
    pitch.motor_data.actual_data.actual_imu_pos = ins->Roll; // 还要取一次负号
    pitch.motor_data.actual_data.actual_gyro = ins->Gyro[1];
    pitch.motor_data.actual_data.actual_ecd_pos =
        pitch.motor_ptr->motor_data_.motor_processed_data.absolute_angle;
    pitch.motor_data.actual_data.actual_spd =
        pitch.motor_ptr->motor_data_.motor_raw_data.feedback_speed;
    pitch.motor_data.actual_data.actual_torgue =
        pitch.motor_ptr->motor_data_.motor_processed_data.torque;

    /*yaw数据更新*/
    yaw.motor_data.actual_data.actual_imu_pos = ins->Yaw;
    yaw.motor_data.actual_data.actual_gyro = yaw.gyro_filter->Calc(ins->Gyro[2]);
    yaw.motor_data.actual_data.actual_ecd_pos = gimbal_maths.loop_fp32_constrain(
        yaw.motor_ptr->motor_data_.motor_processed_data.absolute_angle, -180.0f, 180.0f);
    yaw.motor_data.actual_data.actual_spd =
        yaw.motor_ptr->motor_data_.motor_raw_data.feedback_speed;
    yaw.motor_data.actual_data.actual_torgue =
        yaw.motor_ptr->motor_data_.motor_processed_data.torque;

    /*上下限更新*/
    update_limit();
}

/**
 * @brief 目标值更新
 *
 */
void Gimbal::target_set()
{
    if (gimbal_mode == GIMBAL_MANUAL) {
        if (pitch.motor_ptr->motor_online_flag_) {
            pitch.motor_data.mannal_set +=
                static_cast<float>(robo_cmd->dt7_data_->ch[1] * PITCH_DPI);
        }
        if (yaw.motor_ptr->motor_online_flag_) {
            yaw.motor_data.mannal_set -= static_cast<float>(robo_cmd->dt7_data_->ch[0] * YAW_DPI);
        }
    } else {
        pitch.motor_data.mannal_set = pitch.motor_data.actual_data.actual_imu_pos;
        yaw.motor_data.mannal_set = yaw.motor_data.actual_data.actual_imu_pos;
    }

    /*对锁imu的情况进行上下限幅*/
    user_value_limit(pitch.motor_data.mannal_set, imu_min, imu_max);
    user_value_limit(pitch.motor_data.virtual_set, imu_min, imu_max);

    /*根据模式选择，更新目标值*/
    switch (gimbal_mode) {
        case GIMBAL_MANUAL:
            pitch.motor_data.final_set = pitch.motor_data.mannal_set;
            yaw.motor_data.final_set = yaw.motor_data.mannal_set;
            break;
        case GIMBAL_AUTOATTACK: break;
        case GIMBAL_ZERO_FORCE:
        default:
            pitch.motor_data.final_set = pitch.motor_data.actual_data.actual_imu_pos;
            yaw.motor_data.final_set = yaw.motor_data.actual_data.actual_imu_pos;
            break;
    }
}

/**
 * @brief 云台模式选择
 *
 */
void Gimbal::mode_set()
{
    Behaviour_e last_behav;
    static Behaviour_e rc_behav = gimbal_mode;
    static Behaviour_e kb_behav = gimbal_mode;
    static Behaviour_e vtm_behav = gimbal_mode;

    // dt7
    last_behav = rc_behav;
    switch (robo_cmd->dt7_data_->s2) {
        case DT7_SW_UP: // 遥控器右侧拨杆向上拨，视觉控制云台自瞄
            break;
        case DT7_SW_MID: // 遥控器右侧拨杆向中间拨，手动控制云台
            rc_behav = GIMBAL_MANUAL;
            break;
        case DT7_SW_DOWN: // 往下或其他错误情况：无力模式
        default:
            if (robo_cmd->dt7_data_->s1 != 0 && robo_cmd->dt7_data_->s1 == DT7_SW_MID)
                rc_behav = GIMBAL_MANUAL;
            else if (robo_cmd->dt7_data_->s1 != 0 && robo_cmd->dt7_data_->s1 == DT7_SW_UP)
                rc_behav = GIMBAL_FIRE_TEST;
            else
                rc_behav = GIMBAL_ZERO_FORCE;
            break;
    }

    if (last_behav != rc_behav) // 模式切换
    {
        gimbal_mode = rc_behav;
        if (last_behav == GIMBAL_MANUAL && rc_behav == GIMBAL_FIRE_TEST) // 手瞄切部署，锁编码器
        {
            // pitch_motor->motor_controller_.SetAngleFeedbackPtr(&pitch_motor->motor_data_.motor_processed_data.absolute_angle);
            // pitch_motor->motor_controller_.angle_PID->ECF_PID_CLEAR();
            // pitch_motor_data.mannal_set =
            // pitch_motor->motor_data_.motor_processed_data.absolute_angle;

            // //
            // yaw_motor->motor_controller_.SetAngleFeedbackPtr(&yaw_motor->motor_data_.motor_processed_data.absolute_angle);
            // yaw_motor_data.mannal_set =
            // gimbal_maths.loop_fp32_constrain(yaw_motor->motor_data_.motor_processed_data.absolute_angle,
            // -180.0f, 180.0f); yaw_Angle_PID->ECF_PID_CLEAR();
        } else if (last_behav == GIMBAL_FIRE_TEST &&
                   rc_behav == GIMBAL_MANUAL) // 部署切手瞄，改回imu
        {
            // pitch_motor->motor_controller_.SetAngleFeedbackPtr(&this->pitch_motor_data.actual_data.actual_imu_pos);
            // pitch_motor_data.mannal_set = this->pitch_motor_data.actual_data.actual_imu_pos;
            // pitch_motor->motor_controller_.angle_PID->ECF_PID_CLEAR();
            // //
            // yaw_motor->motor_controller_.SetAngleFeedbackPtr(&this->yaw_motor_data.actual_data.actual_imu_pos);
            // yaw_motor_data.mannal_set = this->yaw_motor_data.actual_data.actual_imu_pos;
            // yaw_Angle_PID->ECF_PID_CLEAR();
        }
    }

    // // 键鼠
    // last_behav = kb_behav;
    // if (dt7->mouse.press_r)
    //     kb_behav = gimbal_set_virtual_Mode();
    // else
    //     kb_behav = GIMBAL_MANUAL;

    // if (last_behav != kb_behav)
    //     gimbal_mode = kb_behav;

    // // VTM
    // last_behav = vtm_behaviour;
    // switch (dt7->vtm.button.mode_sw)
    // {
    // case 0:
    //     vtm_behaviour = GIMBAL_ZERO_FORCE;
    //     break;
    // case 1:
    //     vtm_behaviour = GIMBAL_MANUAL;
    //     break;
    // case 2:
    //     vtm_behaviour = gimbal_set_virtual_Mode();
    // default:
    //     break;
    // }
    // if (last_behav != vtm_behaviour)
    //     gimbal_mode = vtm_behaviour;

    target_set();
}

void Gimbal::crtl_calc()
{
    if (gimbal_mode == GIMBAL_ZERO_FORCE) {
        pitch.final_out = 0.0f;
        yaw.final_out = 0.0f;
        pitch.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        yaw.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    } else {
        /*pitch串级pid计算*/
        pitch.pos_out = pitch.Position_pid.Calc(pitch.motor_data.final_set,
                                                pitch.motor_data.actual_data.actual_imu_pos);
        pitch.spd_out =
            pitch.Speed_pid.Calc(pitch.pos_out, pitch.motor_data.actual_data.actual_gyro);
        pitch.final_out = pitch.spd_out;
        pitch.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, pitch.final_out);

        /*yaw串级pid计算*/
        yaw.yaw_close = gimbal_maths.float_min_distance(
            yaw.motor_data.final_set, yaw.motor_data.actual_data.actual_imu_pos, -180.0f, 180.0f);
        yaw.pos_out = yaw.Position_pid.Calc(yaw.yaw_close);
        yaw.spd_out = yaw.Speed_pid.Calc(yaw.pos_out, yaw.motor_data.actual_data.actual_gyro);
        yaw.final_out = yaw.spd_out;
        yaw.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, yaw.final_out);
    }
}
