/**
 * @file gimbal_task.cpp
 * @author Lann
 * @brief 云台任务
 * @version 0.1
 * @date 2026-01-20
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "gimbal_task.hpp"

void Gimbal_Task(void const* argument)
{
    // 等待 IMU 初始化完成（init_flag 置 1 之后再进入主循环）
    while (Gimbal::GetInstance()->Init()) {
        osDelay(10);
    }

    for (;;) {
        Gimbal::GetInstance()->Loop();
        osDelay(1);
    }
}