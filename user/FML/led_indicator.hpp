/**
 * @file    led_indicator.hpp
 * @brief   LED 指示灯管理模块
 * @details 管理调试心跳灯 (PA7) 和 WS2812 RGB 状态灯 (PA5)。
 *
 *          心跳灯：每 500ms 翻转一次，周期 1 秒。
 *          WS2812 状态灯：
 *          - 正常状态：绿色常亮
 *          - 低电量报警：红色闪烁 (250ms 亮, 250ms 灭)
 */
#ifndef LED_INDICATOR_HPP
#define LED_INDICATOR_HPP

#include "HAL/hal_gpio.hpp"
#include "HDL/ws2812_driver.hpp"

namespace fml {

/// LED 状态枚举
enum class LedState {
    NORMAL,       ///< 正常 - 绿色常亮
    LOW_BATTERY,  ///< 低电量报警 - 红色闪烁
    MUTED,        ///< 静音中 - 红色常亮 (表示低电量但已静音)
    OFF           ///< 关闭
};

/**
 * @class LedIndicator
 * @brief LED 指示灯管理类
 * @details 管理心跳灯闪烁和 WS2812 状态指示。
 */
class LedIndicator {
public:
    /// 心跳灯翻转间隔 (ms)
    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 500;
    /// WS2812 报警闪烁间隔 (ms)
    static constexpr uint32_t BLINK_INTERVAL_MS = 250;

    /**
     * @brief 构造函数
     * @param heartbeatLed 心跳灯 GPIO 引用 (PA7)
     * @param ws2812       WS2812 驱动引用
     */
    LedIndicator(hal::Gpio& heartbeatLed, hdl::WS2812Driver& ws2812)
        : heartbeatLed_(heartbeatLed), ws2812_(ws2812),
          state_(LedState::NORMAL),
          lastHeartbeatTick_(0), lastBlinkTick_(0),
          blinkOn_(false) {}

    /**
     * @brief 初始化 LED 状态
     * @details 心跳灯灭, WS2812 绿色常亮(正常状态)
     */
    void init() {
        heartbeatLed_.setLow();
        ws2812_.setColor(0, 255, 0);  // 绿色 (已受亮度限制)
        state_ = LedState::NORMAL;
    }

    /**
     * @brief 设置 LED 状态
     * @param state 新的 LED 状态
     */
    void setState(LedState state) {
        if (state_ == state) return;
        state_ = state;
        blinkOn_ = false;

        // 切换状态时立即更新 WS2812
        switch (state) {
            case LedState::NORMAL:
                ws2812_.setColor(0, 255, 0);    // 绿色常亮
                break;
            case LedState::LOW_BATTERY:
                ws2812_.setColor(255, 0, 0);    // 红色 (闪烁模式，先亮)
                blinkOn_ = true;
                break;
            case LedState::MUTED:
                ws2812_.setColor(255, 0, 0);    // 红色常亮
                break;
            case LedState::OFF:
                ws2812_.off();
                break;
        }
    }

    /**
     * @brief 周期性更新 (在主循环中调用)
     * @param tick 当前系统时间 (ms, HAL_GetTick)
     */
    void update(uint32_t tick) {
        // 心跳灯处理
        if (tick - lastHeartbeatTick_ >= HEARTBEAT_INTERVAL_MS) {
            lastHeartbeatTick_ = tick;
            heartbeatLed_.toggle();
        }

        // WS2812 闪烁处理 (仅在低电量报警状态)
        if (state_ == LedState::LOW_BATTERY) {
            if (tick - lastBlinkTick_ >= BLINK_INTERVAL_MS) {
                lastBlinkTick_ = tick;
                blinkOn_ = !blinkOn_;
                if (blinkOn_) {
                    ws2812_.setColor(255, 0, 0);  // 红色亮
                } else {
                    ws2812_.off();                 // 灭
                }
            }
        }
    }

private:
    hal::Gpio& heartbeatLed_;       ///< 心跳灯 GPIO 引用
    hdl::WS2812Driver& ws2812_;     ///< WS2812 驱动引用
    LedState state_;                 ///< 当前 LED 状态
    uint32_t lastHeartbeatTick_;     ///< 上次心跳翻转时间
    uint32_t lastBlinkTick_;         ///< 上次闪烁翻转时间
    bool blinkOn_;                   ///< 闪烁状态 (亮/灭)
};

} // namespace fml

#endif // LED_INDICATOR_HPP
