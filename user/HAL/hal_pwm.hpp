/**
 * @file    hal_pwm.hpp
 * @brief   PWM 硬件抽象层封装
 * @details 对 STM32 HAL 库的 TIM PWM 输出进行封装。
 *          TIM1_CH1 配置：Prescaler=0, Period=3999, 即 64MHz/4000 = 16kHz PWM 频率。
 *          用于通过 PWM 模拟 DAC 输出音频信号。
 *
 *          PWM 占空比与音频采样值的关系：
 *          - 8-bit 音频采样值范围: 0~255
 *          - PWM ARR (Period) = 3999
 *          - 映射关系：CCR = sample * 3999 / 255 ≈ sample * 15.68
 *          - 简化为：CCR = sample * 16 (略微偏大但足够精确)
 *          - 或更精确：CCR = (uint32_t)sample * 4000 / 256
 */
#ifndef HAL_PWM_HPP
#define HAL_PWM_HPP

#include "main.h"

namespace hal {

/**
 * @class Pwm
 * @brief PWM 输出抽象类
 * @details 封装 TIM PWM 的启停和占空比设置操作。
 *          支持通过 TIM Update 中断实现周期性占空比刷新。
 */
class Pwm {
public:
    /**
     * @brief 构造函数
     * @param htim    TIM 句柄指针（由 CubeMX 生成的 htim1 等）
     * @param channel PWM 通道 (如 TIM_CHANNEL_1)
     */
    Pwm(TIM_HandleTypeDef* htim, uint32_t channel)
        : htim_(htim), channel_(channel) {}

    /** @brief 启动 PWM 输出 */
    void start() {
        HAL_TIM_PWM_Start(htim_, channel_);
    }

    /** @brief 停止 PWM 输出 */
    void stop() {
        HAL_TIM_PWM_Stop(htim_, channel_);
    }

    /**
     * @brief 启动 PWM 输出并使能 Update 中断
     * @details Update 中断在每个 PWM 周期结束时触发，
     *          用于更新下一个音频采样值的 CCR。
     */
    void startWithIT() {
        HAL_TIM_PWM_Start(htim_, channel_);
        HAL_TIM_Base_Start_IT(htim_);
    }

    /** @brief 停止 PWM 输出并关闭 Update 中断 */
    void stopWithIT() {
        HAL_TIM_Base_Stop_IT(htim_);
        HAL_TIM_PWM_Stop(htim_, channel_);
    }

    /**
     * @brief 直接设置 CCR 值（即占空比原始值）
     * @param ccr 比较寄存器值 (0 ~ Period)
     * @details CCR / (Period+1) = 占空比
     *          当 Period=3999 时，CCR=2000 即 50% 占空比
     */
    void setCompare(uint32_t ccr) {
        __HAL_TIM_SET_COMPARE(htim_, channel_, ccr);
    }

    /**
     * @brief 将 8-bit 音频采样值转换为 CCR 并设置
     * @param sample 8-bit 无符号音频采样值 (0~255)
     * @details 映射公式：CCR = sample * (Period+1) / 256
     *          当 Period=3999 时：CCR = sample * 4000 / 256 = sample * 15.625
     *          使用整数运算：CCR = (uint32_t)sample * 4000 / 256
     */
    void setAudioSample(uint8_t sample) {
        // 将 8-bit 采样值映射到 0~3999 的 CCR 范围
        // CCR = sample * 4000 / 256 = sample * 125 / 8
        uint32_t ccr = static_cast<uint32_t>(sample) * 125UL / 8UL;
        __HAL_TIM_SET_COMPARE(htim_, channel_, ccr);
    }

    /** @brief 获取底层 TIM 句柄 */
    TIM_HandleTypeDef* getHandle() const { return htim_; }

    /** @brief 获取 PWM 通道 */
    uint32_t getChannel() const { return channel_; }

private:
    TIM_HandleTypeDef* htim_;  ///< TIM 句柄指针
    uint32_t channel_;          ///< PWM 通道
};

} // namespace hal

#endif // HAL_PWM_HPP
