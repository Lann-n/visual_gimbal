#include "fire_ctrl.hpp"
namespace MM = Motor_n::MotorBaseDef_n;

void fire_c::Init()
{
    // 右摩擦
    MM::Motor_Base_Config_t right_fire_config =
        MM::Motor_Base_Config_t("right_fire", MM::Motor_Type_euc::M3508)
            .SetControlSetting(
                MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::SPEED_LOOP,
                                            MM::Motor_Reverse_Flag_eb::MOTOR_DIRECTION_REVERSE})
            .SetPIDConfig(alg_n::PidInitConfig_t{},
                          alg_n::PidInitConfig_t{// Speed PID
                                                 .Kp = FIRE_LEFT_KP,
                                                 .Ki = FIRE_LEFT_KI,
                                                 .Kd = FIRE_LEFT_KD,
                                                 .mode = Output_Limit | Feedforward,
                                                 .max_out = FIRE_MAX})
            .SetCANConfig(BSP_n::CanInitConfig_s{
                .can_handle = &hcan1,
                .tx_id = 2, // Slave ID
            })
            .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{.zero_offset = 0.0f,
                                                                       .radius = 0.03f,
                                                                       .ecd2length = 0.0f,
                                                                       .ratio = 1.0f})
            .SetOutputLimit(Current_limit_H_3508, Current_limit_L_3508);
    right_motor = new Motor_n::DjiMotor_n::DjiDriver_c(right_fire_config);
    right_motor->motor_controller_.SetSpeedFeedbackPtr(
        &right_motor->motor_data_.motor_raw_data.feedback_speed);
    right_motor_data = &right_motor->get_base();

    // 左摩擦
    MM::Motor_Base_Config_t left_fire_config =
        MM::Motor_Base_Config_t("left_fire", MM::Motor_Type_euc::M3508)
            .SetControlSetting(MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::SPEED_LOOP})
            .SetPIDConfig(alg_n::PidInitConfig_t{},
                          alg_n::PidInitConfig_t{// Speed PID
                                                 .Kp = FIRE_LEFT_KP,
                                                 .Ki = FIRE_LEFT_KI,
                                                 .Kd = FIRE_LEFT_KD,
                                                 .mode = Output_Limit | Feedforward,
                                                 .max_out = FIRE_MAX})
            .SetCANConfig(BSP_n::CanInitConfig_s{
                .can_handle = &hcan1,
                .tx_id = 1, // Slave ID
            })
            .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{.zero_offset = 0.0f,
                                                                       .radius = 0.03f,
                                                                       .ecd2length = 0.0f,
                                                                       .ratio = 1.0f})
            .SetOutputLimit(Current_limit_H_3508, Current_limit_L_3508);
    left_motor = new Motor_n::DjiMotor_n::DjiDriver_c(left_fire_config);
    left_motor->motor_controller_.SetSpeedFeedbackPtr(
        &left_motor->motor_data_.motor_raw_data.feedback_speed);
    left_motor_data = &left_motor->get_base();

    // 拨弹盘
    MM::Motor_Base_Config_t pluck_config =
        MM::Motor_Base_Config_t("pluck", MM::Motor_Type_euc::M2006)
            .SetControlSetting(MM::Motor_Control_Setting_t{MM::Closeloop_Type_euc::OPEN_LOOP})

            .SetCANConfig(BSP_n::CanInitConfig_s{
                .can_handle = &hcan2,
                .tx_id = 6, // Slave ID
            })
            .SetMechanicalParams(MM::Motor_Data_t::Motor_Fixed_Param_t{.zero_offset = 0.0f,
                                                                       .radius = 0.03f,
                                                                       .ecd2length = 0.0f,
                                                                       .ratio = 1.0f})
            .SetOutputLimit(Current_limit_H_2006, Current_limit_L_2006);
    pluck_motor = new Motor_n::DjiMotor_n::DjiDriver_c(pluck_config);
    pluck_motor_data = &pluck_motor->get_base();

    // 单发控制初始化
    alg_n::PidInitConfig_t pid_pluck_p_cof = {.Kp = FIRE_PLUCK_P_KP,
                                              .Ki = FIRE_PLUCK_P_KI,
                                              .Kd = FIRE_PLUCK_P_KD,
                                              .ActualValueSource = nullptr,
                                              .mode = Output_Limit,
                                              .max_out = FIRE_PLUCK_P_MAX};

    alg_n::PidInitConfig_t pid_pluck_s_cof = {.Kp = FIRE_PLUCK_S_KP,
                                              .Ki = FIRE_PLUCK_S_KI,
                                              .Kd = FIRE_PLUCK_S_KD,
                                              .ActualValueSource = nullptr,
                                              .mode = Output_Limit | Integral_Limit,
                                              .max_out = FIRE_PLUCK_S_MAX,
                                              .max_Ierror = FIRE_PLUCK_S_IMAX};

    // 连发控制初始化
    alg_n::PidInitConfig_t pid_fire_cof = {
        .Kp = FIRE_FIRE_KP,
        .Ki = FIRE_FIRE_KI,
        .Kd = FIRE_FIRE_KD,
        .ActualValueSource = nullptr,
        .mode = Output_Limit,
        .max_out = PLUCK_MAX,
    };
    semi_pos_pid.Init(pid_pluck_p_cof);
    semi_spd_pid.Init(pid_pluck_s_cof);
    auto_pid.Init(pid_fire_cof);
    semi_pos_pid.ChangeActValSource(&pluck_motor->motor_data_.motor_processed_data.total_ecd);
    semi_spd_pid.ChangeActValSource(&pluck_motor->motor_data_.motor_raw_data.feedback_speed);
    auto_pid.ChangeActValSource(&pluck_motor->motor_data_.motor_raw_data.feedback_speed);
    speed_filter = new alg_n::FirstOrderFilter_c(1.0f);

    enable_motor();
    robo_cmd = RoboCmd_c::GetInstance();
    dwt = BSP_n::DWT_c::Get_DwtInstance();
}

void fire_c::Loop()
{

    mode_set();
    ctrl_fire_motor();
}

/**
 * @brief 摩擦轮无力
 * @enter 状态位为NO_FIRE，或者主动调用顺带切换为NO_FIRE
 */
void fire_c::fire_zero_force()
{
    fire_mode = NO_FIRE;
    motor_set_value.left_motor_speed_set = 0;
    motor_set_value.right_motor_speed_set = 0;
    left_motor->Disable();
    right_motor->Disable();
    left_motor->motor_data_.clear();
    right_motor->motor_data_.clear();
    left_motor->SetMotorOutputFix(0);
    right_motor->SetMotorOutputFix(0);
}

/**
 * @brief 拨弹盘无力
 *
 */
void fire_c::pluck_zero_force()
{
    // fire_mode = NO_FIRE;
    motor_set_value.pluck_motor_semi_set = 0.0f;
    motor_set_value.pluck_motor_auto_set = 0.0f;
    pos_out = 0.0f;
    pluck_motor->motor_data_.motor_processed_data.clear();
}

void fire_c::reset_pluck()
{
    fire_mode = READY;
    semi_pos_pid.Clear();
    semi_spd_pid.Clear();
    auto_pid.Clear();
    pos_out = 0.0f;
    final_out = 0.0f;
    pluck_zero_force();
}

/**
 * @brief 状态机更新
 *
 */
void fire_c::mode_set()
{
    static keyboard_util::KeyCode rc_fire_mode;
    rc_fire_mode.update(
        robo_cmd->dt7_data_->ch[4] < -300,
        [this]() {
            if (fire_mode != NO_FIRE)
                fire_mode = NO_FIRE;
            else if (fire_mode != READY || fire_mode != STUCK) {
                fire_mode = READY;
                pluck_zero_force();
            }
        },
        nullptr);
    static keyboard_util::KeyCode pluck_ctrl; // 短按单发，长按全自动
    pluck_ctrl.update((robo_cmd->dt7_data_->ch[4] > 500), nullptr, nullptr,
                      [this] {
                          if (fire_mode == STUCK || fire_mode == NO_FIRE)
                              return;
                          else {
                              fire_mode = AUTO;
                              motor_set_value.pluck_motor_auto_set = fire_rate(25);
                          }
                      },
                      [this] {
                          if (fire_mode == STUCK || fire_mode == NO_FIRE)
                              return;
                          else {
                              fire_mode = SEMI;
                              motor_set_value.pluck_motor_semi_set =
                                  AN_BULLET * SEMI_NUM + dead_erro;
                          }
                      },
                      80, 550);
    /*连发释放*/
    if ((fire_mode == AUTO) &&
        (robo_cmd->dt7_data_->ch[4] > -10 && robo_cmd->dt7_data_->ch[4] < 100)) {
        fire_mode = READY;
        pluck_zero_force();
    }
    /*单发释放*/
    if ((fire_mode == SEMI) &&
        (abs(motor_set_value.pluck_motor_semi_set -
             pluck_motor->motor_data_.motor_processed_data.total_ecd) < 3000) &&
        (abs(pluck_motor->motor_data_.motor_raw_data.feedback_speed) < 1300)) {
        fire_mode = READY;
        pluck_zero_force();
    }
    if (fire_mode == READY) {
        motor_set_value.pluck_motor_semi_set = 0.0f;
        motor_set_value.pluck_motor_auto_set = 0.0f;
        // pluck_motor->motor_data_.clear();
        // final_out = auto_pid.Calc(motor_set_value.pluck_motor_auto_set,
        //                           pluck_motor->motor_data_.motor_raw_data.feedback_speed);
        // pluck_motor->SetMotorOutputFix(-final_out);
    }
    check_stuck(); // 堵转检测
}

/**
 * @brief 检测摩擦轮是否转动允许开火
 *
 */
bool fire_c::check_allow_fire()
{
    static float last_ms;
    static float dt;
    // 摩擦轮转速小于某个值且不处于不发射状态位，则切换为不发射状态，并进行相应保护处理
    if ((left_motor->motor_data_.motor_raw_data.feedback_speed <
             motor_set_value.left_motor_speed_set ||
         right_motor->motor_data_.motor_raw_data.feedback_speed <
             motor_set_value.right_motor_speed_set) &&
        fire_mode != NO_FIRE) {
        if (dt > 2000) {
            fire_zero_force(); // 执行摩擦轮无力函数，设定为不发射状态位
            pluck_zero_force();
            dt = 0;
            return 1;
        } else if (dt == 0) {
            last_ms = dwt->GetTimeline_ms();
        } else {
            dt = dwt->GetTimeline_ms() - last_ms;
        }
    }
    return 0;
}

void fire_c::ctrl_fire_motor()
{
    if (check_allow_fire()) return;
    if (fire_mode == READY) {
        left_motor->Enable();
        right_motor->Enable();
        motor_set_value.left_motor_speed_set = (fire_speed * FIRESPEED_TO_MOTORRPM);
        motor_set_value.right_motor_speed_set = (fire_speed * FIRESPEED_TO_MOTORRPM);
    } else if (fire_mode == NO_FIRE) {
        fire_zero_force();
        pluck_zero_force();
    }

    left_motor->DjiMotorSetRef(motor_set_value.left_motor_speed_set);
    right_motor->DjiMotorSetRef(motor_set_value.right_motor_speed_set);
}

void fire_c::pluck_ctrl()
{
    //其实这个地方好像没什么用，后面那个解决了
    static mode last_mode = NO_FIRE;
    if (last_mode != fire_mode) {
        semi_pos_pid.Clear();
        semi_spd_pid.Clear();
        auto_pid.Clear();
        last_mode = fire_mode;
    }
    speed_filter->Calc(pluck_motor->motor_data_.motor_raw_data.feedback_speed);
    if (fire_mode == SEMI) {
        pos_out = semi_pos_pid.Calc(motor_set_value.pluck_motor_semi_set,
                                    pluck_motor->motor_data_.motor_processed_data.total_ecd);
        final_out =
            semi_spd_pid.Calc(pos_out, pluck_motor->motor_data_.motor_raw_data.feedback_speed);
        pluck_motor->SetMotorOutputFix(final_out);
    } else {
        final_out = auto_pid.Calc(motor_set_value.pluck_motor_auto_set,
                                  pluck_motor->motor_data_.motor_raw_data.feedback_speed);
        pluck_motor->SetMotorOutputFix(-final_out);
        // sb拨盘零速控不住的一直晃沃日尼玛
        if ((fire_mode == NO_FIRE || fire_mode == READY) &&
            (abs(pluck_motor->motor_data_.motor_raw_data.feedback_speed - 0) < 2000)) {
            auto_pid.Clear();
            final_out = 0;
            pluck_motor->SetMotorOutputFix(0);
        }
    }
}

/**
 * @brief 所有模式都可使用
 *
 */
void fire_c::all_fire_ctrl()
{
    semi_ctrl();
    // free_semi();
    auto_ctrl();
}

void fire_c::semi_ctrl()
{
    if (fire_mode == SEMI) {
        motor_set_value.pluck_motor_semi_set = AN_BULLET * SEMI_NUM + dead_erro;
        pos_out = semi_pos_pid.Calc(motor_set_value.pluck_motor_semi_set,
                                    pluck_motor->motor_data_.motor_processed_data.total_ecd);
        final_out =
            semi_spd_pid.Calc(pos_out, pluck_motor->motor_data_.motor_raw_data.feedback_speed);
        pluck_motor->SetMotorOutputFix(final_out);
    }
}

void fire_c::auto_ctrl()
{
    if (fire_mode == AUTO) {
        motor_set_value.pluck_motor_auto_set = fire_rate(25);
        final_out = auto_pid.Calc(motor_set_value.pluck_motor_auto_set,
                                  pluck_motor->motor_data_.motor_raw_data.feedback_speed);
        pluck_motor->SetMotorOutputFix(-final_out); // 沟槽的要取反
    }
}

void fire_c::ready_ctrl()
{
    if (fire_mode == READY) {
        motor_set_value.pluck_motor_semi_set = 0.0f;
        motor_set_value.pluck_motor_auto_set = 0.0f;
        // pluck_motor->motor_data_.clear();
        final_out = auto_pid.Calc(motor_set_value.pluck_motor_auto_set,
                                  pluck_motor->motor_data_.motor_raw_data.feedback_speed);
        pluck_motor->SetMotorOutputFix(-final_out);
    }
}

void fire_c::check_stuck()
{
    static keyboard_util::KeyCode stuck;

    stuck.update((abs(pluck_motor->motor_data_.motor_raw_data.force_feedback) > 8000) &&
                     (abs(pluck_motor->motor_data_.motor_raw_data.feedback_speed) < 500),
                 nullptr,
                 [this] {
                     if (fire_mode == STUCK) pluck_zero_force();
                 },
                 [this] {
                     if (fire_mode != STUCK) {
                         fire_mode = STUCK;
                         pluck_zero_force();
                     }
                 },
                 nullptr, 0, 2500);
}