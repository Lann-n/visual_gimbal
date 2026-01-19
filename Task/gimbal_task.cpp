#include "gimbal_task.hpp"

void Gimbal_Task(void const * argument)
{
    // 等待 IMU 初始化完成（init_flag 置 1 之后再进入主循环）
    while(Gimbal::GetInstance()->Init())
    {
        osDelay(10);
    }

    for(;;)
    {

    }
}