#pragma once

#include "gimbal.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

void Gimbal_Task(void const* argument);
}
