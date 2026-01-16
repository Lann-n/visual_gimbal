#ifndef __ECF_CONFIGLIST_H
#define __ECF_CONFIGLIST_H

/**************cmd_config_start********************** */
#ifdef USE_REFEREE
    #ifdef STM32H723xx
    #define REFEREE_USART huart2
    #elif STM32F405xx
    #define REFEREE_USART huart6
    #endif
#endif

#ifdef USE_VT13
    #ifdef STM32H723xx
    #define VTM_USART huart2
    #elif STM32F405xx
    #define VTM_USART huart6
    #endif
#endif

#ifdef USE_DR16
    #ifdef STM32H723xx
    #define DT7_USART huart5
    #elif STM32F405xx
    #define DT7_USART huart3
    #endif
#endif

#define Clear_DT7_SW_R 0U
#define Clear_DT7_SW_L 0U
/**************cmd_config_end********************** */

/**************can_config_start********************** */
#ifdef STM32H723xx
    #define USE_FDCAN1
    #define USE_FDCAN2
    #define USE_FDCAN3
#else
    #define USE_CAN1
    #define USE_CAN2
#endif // USE_H7_if_or_not
/**************cmd_config_end********************** */

/**************power_config_start********************** */
#ifdef USE_POWER_CTRL
    #ifdef STM32H723xx
    #define CAP_FDCAN &hfdcan3
    #elif STM32F405xx
    #define CAP_CAN &hcan3
    #endif
    #define CAP_TX_ID 0x210
    #define CAP_RX_ID 0x211
#endif 
/**************power_config_end********************** */

/**************imu_config_start********************** */
#ifdef USE_IMU

    #define Heater_Kp 7.f
    #define Heater_Ki 0.f
    #define Heater_Kd 0.f
    #define Heater_maxIerror 10

    #define Heater_forward 15//加热器前馈值
    #define Init_temp 35.f//初始化前温度
    #define Target_temp 40.f//目标温度
    #define Max_temp 80.f//最高温度
    #define Preheat_Timeout 20*1000//预热超时时间，单位ms
    //TIM通道
    #define IMU_HEATER_TIM &htim2
    #define IMU_HEATER_TIM_CHANNEL TIM_CHANNEL_3
    /************************************ */
    //spi
    #define BMI088_SPI &hspi1
    //
    #define isHeaterEnabled 0 //是否启用加热器
    #define isCalibrate 1 //是否启用在线校准
    /***********离线校准数据*************** */
    // 需手动修改
    #define GxOFFSET -0.0f
    #define GyOFFSET -0.0f
    #define GzOFFSET 0.0f
    #define gNORM 9.6f

    /***********加速度计修正安装误差*************** */
    #define CENTER_IMU_rx 0.f // imu距离 机体中心/旋转中心/质心 在 imu 坐标系 x轴方向上的 距离 单位m
    #define CENTER_IMU_ry 0.f // imu距离 机体中心/旋转中心/质心 在 imu 坐标系 y轴方向上的 距离 单位m
    #define CENTER_IMU_rz 0.f // imu距离 机体中心/旋转中心/质心 在 imu 坐标系 z轴方向上的 距离 单位m
    /************角度安装修正******************** */
    //偏差多少填多少
    #define IMU_Pitch_Offset 0.f // imu安装俯仰角修正 单位deg 
    #define IMU_Roll_Offset 0.f  // imu安装横滚角修正 单位deg

#endif 
/**************imu_config_end********************** */

/**************ws2812_config_start********************** */
#ifdef USE_WS2812

// SPI
#define MYSPI hspi2
// 编码 0 : 11000000
#define CODE_0        0xC0
// 编码 1 : 11111000
#define CODE_1        0xF8
// ws2812b灯珠数量
#define WS2812_AMOUNT 1
// RGB bit
#define RGB_BIT 24
// 
#define SHIT_SPIHAL 1
// 复位：低电平时间 > 80us
// SPI 6.4MHz 时（1Byte≈1.25us），90/1.25 = 72Byte 低电平，单个SCK周期： 6.4e6Hz ≈ 156.25 ns，1 Byte = 8 bit（SPI 8-bit 数据帧）：8 * 156.25 ns = 1250 ns = 1.25 µs
#define WS2812_RESET_BYTES 72u
// 帧率
#define FPS_MS 20u

#endif 
/**************ws2812_config_end********************** */

#endif