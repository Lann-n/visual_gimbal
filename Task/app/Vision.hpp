#ifndef VISION_H
#define VISION_H

#include "safe_task.hpp"

extern "C" {
// #include "fifo.h"
#include "usbd_cdc_if.h"
#include "visual_config.h"
}

#define VIRTUAL_DATA_LEN 37

void Virtual_Init(void);
void Virtual_recive(void);
void Virtual_send(uint8_t aim_color,
                  float Pitch,
                  float Yaw,
                  float Roll,
                  float _bullet_speed,
                  uint8_t _reset_tracker,
                  uint8_t _now_mode,
                  uint8_t fired);
void Virtual_Clear();
Visual_Rx_t* Get_virtual_recive_ptr();

#endif