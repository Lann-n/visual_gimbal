#pragma once

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"

    void Gimbal_Task(void const * argument);
}