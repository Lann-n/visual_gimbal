/**
 * @file KeyboardUtil.hpp
 * @author Lann
 * @brief 
 * @version V1.0 好用爱用
 * @version V1.1 适配了新框架
 * @version V1.2 新增清除函数
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
//
// Created by Lann on 2025/2/25.
//


#ifndef KEYBOARDUTIL_HPP
#define KEYBOARDUTIL_HPP

#include <functional>
#include <cstdint>
#include "bsp_dwt.hpp"

namespace keyboard_util {

    using OnKeyDown = std::function<void()>;
    using OnKeyUp = std::function<void()>;
    using OnKeyLongPress = std::function<void()>;
    using OnKeyClick = std::function<void()>;//新增单击回调检测

    class KeyCode {
    private:
        bool is_last_pressed = false;
        uint32_t press_start_time = 0;  // 记录按下开始时间
        bool long_press_triggered = false;  // 防止重复触发长按

    public:
        
        inline void update(const uint16_t state, const OnKeyDown& on_key_down, const OnKeyUp& on_key_up) {
            bool is_pressed = state == 1;
            if (!is_last_pressed && is_pressed) // 按键被按下瞬间
            {
            if (on_key_down != nullptr) {
                on_key_down();
            }
            }
            else if (is_last_pressed && !is_pressed) // 按键被松开瞬间
            {
                if (on_key_up != nullptr) {
                    on_key_up();
                }
            }
            is_last_pressed = is_pressed;
        }

        // 新重载版本添加长按检测
        inline void update(const uint16_t state,
                            const OnKeyDown& on_key_down,
                            const OnKeyUp& on_key_up,
                            const OnKeyLongPress& on_key_long_press,
                            const OnKeyClick& on_key_click,
                            uint32_t thresholdMin_ms = 0,
                            uint32_t thresholdMax_ms = 500) {  // 长按阈值默认500ms
            bool is_pressed = state == 1;
            BSP_n::DWT_c *KeyDwtTime = BSP_n::DWT_c::Get_DwtInstance(); // 获取DWT实例
            auto current_time = KeyDwtTime->GetTimeline_ms();
            // 触发瞬时事件
            if (!is_last_pressed && is_pressed) {
                press_start_time = current_time;
                long_press_triggered = false;
                if (on_key_down != nullptr) on_key_down();
            }
            else if (is_last_pressed && !is_pressed) {
                // 单击检测条件：未触发长按 且 按压时间小于阈值
                if (!long_press_triggered && 
                    (current_time - press_start_time <= thresholdMax_ms) &&
                    (current_time - press_start_time >= thresholdMin_ms) &&
                    (on_key_click != nullptr)) {
                    on_key_click();
                }
                press_start_time = 0;
                if (on_key_up != nullptr) on_key_up();
            }

            // 长按检测
            if (is_pressed && !long_press_triggered) {
                if (KeyDwtTime->GetTimeline_ms() - press_start_time >= thresholdMax_ms) {
                    if (on_key_long_press != nullptr) on_key_long_press();
                        long_press_triggered = true;  // 防止重复触发
                }
            }

            is_last_pressed = is_pressed;
        }

        inline void clear()
        {
            is_last_pressed = false;
            press_start_time = 0;
            long_press_triggered = false;
        }
    };

}// namespace keyboard_util

#endif //KEYBOARDUTIL_HPP
