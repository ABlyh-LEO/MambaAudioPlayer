/**
 * @file    amp_control.hpp
 * @brief   PAM8302A 音频功放控制驱动
 * @details 通过 GPIO 控制 PAM8302A 的 SHDN 引脚实现功放使能/关断。
 *          PA3 (AMP_SHDN): 高电平=工作, 低电平=静音关断(低功耗)
 *
 *          PAM8302A 参数：
 *          - 工作电压：2.0V ~ 5.5V
 *          - 输出功率：2.5W @ 5V, 4Ω (本设计使用 3.3V 供电)
 *          - 关断电流：<1μA
 *          - 增益：固定 12dB
 */
#ifndef AMP_CONTROL_HPP
#define AMP_CONTROL_HPP

#include "HAL/hal_gpio.hpp"

namespace hdl {

/**
 * @class AmpControl
 * @brief 音频功放 (PAM8302A) 使能控制类
 * @details 仅控制 SHDN 引脚的高低电平。
 *          上电默认为关断状态 (SHDN=低)。
 */
class AmpControl {
public:
    /**
     * @brief 构造函数
     * @param shdnPin SHDN 引脚对象引用 (PA3)
     */
    explicit AmpControl(hal::Gpio& shdnPin)
        : shdn_(shdnPin) {}

    /**
     * @brief 使能功放（开始输出音频）
     * @details 拉高 SHDN 引脚，功放进入工作模式。
     *          建议在设置好 PWM 输出后再使能，避免上电噪声。
     */
    void enable() {
        shdn_.setHigh();
    }

    /**
     * @brief 关断功放（静音，低功耗）
     * @details 拉低 SHDN 引脚，功放进入关断模式。
     *          关断电流 <1μA，音频输出完全静音。
     */
    void disable() {
        shdn_.setLow();
    }

    /**
     * @brief 获取功放当前状态
     * @return true=工作中; false=已关断
     */
    bool isEnabled() const {
        return shdn_.read();
    }

private:
    hal::Gpio& shdn_;  ///< SHDN 引脚引用
};

} // namespace hdl

#endif // AMP_CONTROL_HPP
