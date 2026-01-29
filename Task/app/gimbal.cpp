#include "gimbal.hpp"
#include <cstdint>

namespace MM = Motor_n::MotorBaseDef_n;

void Gimbal::Decode_Chassis_Data(BSP_n::Can_c* instance)
{
    if (instance->GetRxId() != CHASSIS_ID) return;
    memcpy(&chassis_data, instance->rx_buff_, 8);
    fire.shoot_msg.shoot_barrel_heat_current = chassis_data.com_packet_data.shooter_heat;
    fire.shoot_msg.shoot_barrel_heat_limit = chassis_data.com_packet_data.heat_limit;
    fire.shoot_msg.shoot_bullet_speed = chassis_data.com_packet_data.bullet_speed / 100.0f;
    fire.shoot_msg.aim_color = chassis_data.com_packet_data.aim_color;
    fire.shoot_msg.robot_level = chassis_data.com_packet_data.robot_level;
    chassis_yaw_dot = (chassis_data.com_packet_data.yaw_dot - 1024.0f) / 1024.0f * 20.0f;
}

void Gimbal::Gimbal2Chassis()
{
    CONTROL_SEND_HZ(4); // 250Hz

    static uint8_t gimbal_status = 2;
    if (gimbal_mode == GIMBAL_MANUAL)
        gimbal_status = 3;
    else if (gimbal_mode == GIMBAL_AUTOATTACK)
        gimbal_status = 1;
    else
        gimbal_status = 2;
    gimbal_data.com_packet_data.rc_channel2 = robo_cmd->dt7_data_->ch[2];
    gimbal_data.com_packet_data.rc_channel3 = robo_cmd->dt7_data_->ch[3];
    gimbal_data.com_packet_data.rc_dial = robo_cmd->dt7_data_->ch[4];
    gimbal_data.com_packet_data.rc_s1 = robo_cmd->dt7_data_->s1; // 后续要做跟vt13的处理
    gimbal_data.com_packet_data.rc_s2 = gimbal_status;
    gimbal_data.com_packet_data.id = (visual_data->distance <= 0 ? 0 : 1);
    gimbal_data.com_packet_data.fric_onoff = (fire.fire_mode != fire_c::NO_FIRE ? 1 : 0);
    gimbal_data.com_packet_data.gimbal_mode = gimbal_mode;
    gimbal_data.com_packet_data.fire_mode = (fire.fire_mode == fire_c::SEMI ? 1 : 0);
    gimbal_data.com_packet_data.aim_mode = visual_tx_data.now_mode;
    gimbal_data.com_packet_data.fire_speed = (uint8_t)((fire_speed(25) - 20.0f) / 0.5);

    memcpy(broad_com->tx_buff_, &gimbal_data, 8);
    broad_com->Transmit(1);
}

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

    dwt = BSP_n::DWT_c::Get_DwtInstance();
    Virtual_Init();
    visual_data = Get_virtual_recive_ptr();
    fire.Init();
    broad_com =
        new BSP_n::Can_c(&hcan2, GIMBAL_ID, CHASSIS_ID, CAN_ID_STD,
                         [this](BSP_n::Can_c* instance) { this->Decode_Chassis_Data(instance); });
    visual_contrl.Init(
        &visual_data->pitch, &visual_data->pitch_gyro, &visual_data->pitch_omg,
        &pitch.motor_data.actual_data.actual_imu_pos, &pitch.motor_data.actual_data.actual_gyro,
        &visual_data->yaw, &visual_data->yaw_gyro, &visual_data->yaw_omg,
        &yaw.motor_data.actual_data.actual_imu_pos, &yaw.motor_data.actual_data.actual_gyro);
    return 0;
}
static uint8_t vision_mode = 2; // 视觉模式，暂时这么写
void Gimbal::Loop()
{
    Virtual_recive(); // 获取视觉数据

    Virtual_send(fire.shoot_msg.aim_color, ins->Pitch, ins->Yaw, ins->Roll,
                 fire.shoot_msg.shoot_bullet_speed, 0, vision_mode, fire.fired);
    Gimbal2Chassis();
    update_feedback();
    mode_set();
    last_time = dwt->GetTimeline_ms();
    pitch_dif = abs(visual_data->pitch - pitch.motor_data.actual_data.actual_imu_pos);
    yaw_dif = abs(visual_data->yaw - yaw.motor_data.actual_data.actual_imu_pos);
    bool gimbal_is_close = false;
    // 当视觉识别到目标时才开火
    if (vision_mode == 1|| vision_mode==0) // 自瞄模式
    {
        gimbal_is_close = ((pitch_dif < pitch_dif_target) && (yaw_dif < yaw_dif_target));
    } else if (vision_mode == 2 || vision_mode == 3) {
        float range = fabs(atan2(TARGET_RADIUS, visual_data->distance)) * 180.0 / M_PI;
        float pitch_range = user_maths_c().max_abs(range, 1.5);
        float yaw_range = user_maths_c().max_abs(range, 1.5);
        gimbal_is_close = (fabs(pitch_dif) <= pitch_range && fabs(yaw_dif) <= yaw_range);
    }
    fire.Loop(gimbal_mode, gimbal_is_close);
    crtl_calc();
    dt = dwt->GetTimeline_ms() - last_time;
    pitch.motor_ptr->Transmit(1);
    yaw.motor_ptr->Transmit(1);
    task_cnt++;
}

/**
 * @brief 基于pitch编码值更新imu上下限
 *
 */
void Gimbal::update_limit()
{
    reduce_angle = (PITCH_ANGLE_MIN + 2.0f) -
                   pitch.motor_ptr->motor_data_.motor_processed_data.relative_angle_180;
    increase_angle = (PITCH_ANGLE_MAX - 2.0f) -
                     pitch.motor_ptr->motor_data_.motor_processed_data.relative_angle_180;
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

    pitch.motor_data.virtual_set = visual_data->pitch;
    yaw.motor_data.virtual_set = visual_data->yaw;

    /*对锁imu的情况进行上下限幅*/
    user_value_limit(pitch.motor_data.mannal_set, imu_min, imu_max);
    user_value_limit(pitch.motor_data.virtual_set, imu_min, imu_max);

    /*根据模式选择，更新目标值*/
    switch (gimbal_mode) {
        case GIMBAL_MANUAL:
            pitch.motor_data.final_set = pitch.motor_data.mannal_set;
            yaw.motor_data.final_set = yaw.motor_data.mannal_set;
            break;
        case GIMBAL_AUTOATTACK:
            pitch.motor_data.final_set = pitch.motor_data.virtual_set;
            yaw.motor_data.final_set = yaw.motor_data.virtual_set;
            break;
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
    static Behaviour_e last_behav;
    static Behaviour_e rc_behav = gimbal_mode;
    static Behaviour_e kb_behav = gimbal_mode;
    static Behaviour_e vtm_behav = gimbal_mode;

    // dt7
    last_behav = rc_behav;
    switch (robo_cmd->dt7_data_->s2) {
        case DT7_SW_UP: // 遥控器右侧拨杆向上拨，视觉控制云台自瞄
            if (visual_data->distance == 0 || visual_data->distance == -1) {
                rc_behav = GIMBAL_MANUAL;
            } else {
                rc_behav = GIMBAL_AUTOATTACK;
            }
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
        if (gimbal_mode == GIMBAL_AUTOATTACK) {
            // pitch.Position_pid.UpdateParam(pitch_visual_angle_config);
            // pitch.Speed_pid.UpdateParam(pitch_visual_speed_config);
            // yaw.Position_pid.UpdateParam(yaw_visual_angle_config);
            // yaw.Speed_pid.UpdateParam(yaw_visual_speed_config);
            pitch.Visual_angle_pid.Clear();
            pitch.Visual_speed_pid.Clear();
            yaw.Visual_angle_pid.Clear();
            yaw.Visual_speed_pid.Clear();
        } else {
            // pitch.Position_pid.UpdateParam(pitchMotorConfig.angle_PID);
            // pitch.Speed_pid.UpdateParam(pitchMotorConfig.speed_PID);
            // yaw.Position_pid.UpdateParam(yawMotorConfig.angle_PID);
            // yaw.Speed_pid.UpdateParam(yawMotorConfig.speed_PID);
            pitch.Position_pid.Clear();
            pitch.Speed_pid.Clear();
            yaw.Position_pid.Clear();
            yaw.Speed_pid.Clear();
        }
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
    } else if (gimbal_mode == GIMBAL_AUTOATTACK) {
        // 两种控制模式选择
        if (visual_contrl.use_it == true) {
            pitch.final_out = visual_contrl.pitch_calc();
            yaw.final_out = visual_contrl.yaw_calc();
        } else {
            /*pitch串级pid计算*/
            pitch.pos_out = pitch.Visual_angle_pid.Calc(pitch.motor_data.final_set,
                                                        pitch.motor_data.actual_data.actual_imu_pos,
                                                        visual_data->pitch_omg);
            pitch.spd_out = pitch.Visual_speed_pid.Calc(
                pitch.pos_out, pitch.motor_data.actual_data.actual_spd / 9.55f,
                visual_data->pitch_gyro);
            G_out = this->gravity_compensation_f(pitch.motor_data.actual_data.actual_imu_pos);
            pitch.final_out = (pitch.spd_out + G_out);

            /*yaw串级pid计算*/
            yaw.yaw_close = gimbal_maths.float_min_distance(
                                yaw.motor_data.final_set, yaw.motor_data.actual_data.actual_imu_pos,
                                -180.0f, 180.0f) +
                            yaw.motor_data.actual_data.actual_imu_pos;
            yaw.pos_out = yaw.Visual_angle_pid.Calc(
                yaw.yaw_close, yaw.motor_data.actual_data.actual_imu_pos, visual_data->yaw_omg);
            yaw_fd_out = (this->yaw_feedforward * this->chassis_yaw_dot); // 顺从了
            yaw.spd_out = yaw.Visual_speed_pid.Calc((yaw.pos_out + yaw_fd_out),
                                                    yaw.motor_data.actual_data.actual_spd / 9.55f,
                                                    visual_data->yaw_gyro);
            yaw.final_out = yaw.spd_out;

            pitch.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, pitch.final_out);
            yaw.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, yaw.final_out);
        }

    } else {
        /*pitch串级pid计算*/
        pitch.pos_out = pitch.Position_pid.Calc(pitch.motor_data.final_set,
                                                pitch.motor_data.actual_data.actual_imu_pos);
        pitch.spd_out =
            pitch.Speed_pid.Calc(pitch.pos_out, pitch.motor_data.actual_data.actual_gyro);
        G_out = this->gravity_compensation_f(pitch.motor_data.actual_data.actual_imu_pos);
        pitch.final_out = pitch.spd_out + G_out;

        /*yaw串级pid计算*/
        yaw.yaw_close = gimbal_maths.float_min_distance(yaw.motor_data.final_set,
                                                        yaw.motor_data.actual_data.actual_imu_pos,
                                                        -180.0f, 180.0f) +
                        yaw.motor_data.actual_data.actual_imu_pos;
        yaw.pos_out =
            yaw.Position_pid.Calc(yaw.yaw_close, yaw.motor_data.actual_data.actual_imu_pos);
        yaw.spd_out = yaw.Speed_pid.Calc(yaw.pos_out, yaw.motor_data.actual_data.actual_gyro);
        yaw.final_out = yaw.spd_out;

        pitch.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, pitch.final_out);
        yaw.motor_ptr->SetMITData(0.0f, 0.0f, 0.0f, 0.0f, yaw.final_out);
    }
}
