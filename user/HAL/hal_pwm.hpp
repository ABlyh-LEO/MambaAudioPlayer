/**
 * @file    hal_pwm.hpp
 * @brief   PWM 硬件抽象层封装
 * @details 对 STM32 HAL 库的 TIM PWM 输出进行封装。
 *
 *          TIM1_CH1 配置 (高频 PWM + RCR 分频实现精确采样率):
 *          - Prescaler = 0
 *          - Period (ARR) = 249 → PWM 频率 = 64MHz / 250 = 256kHz
 *          - RepetitionCounter (RCR) = 7 → Update 中断频率 = 256kHz / 8 = 32kHz
 *
 *          设计原理：
 *          PWM 频率 (256kHz) 远高于 RC 低通滤波器截止频率 (1kΩ+10nF → f_c≈15.9kHz)，
 *          滤波器在 256kHz 处衰减约 -24dB，能有效去除 PWM 开关纹波。
 *          RCR=7 使得 TIM Update 中断 (用于更新音频采样值) 恰好以 32kHz 触发。
 *
 *          PWM 占空比与音频采样值的关系：
 *          - 8-bit 音频采样值范围: 0~255
 *          - PWM ARR (Period) = 249，共 250 个计数级
 *          - 映射关系：CCR = sample * 250 / 256
 *          - 整数运算：CCR = (uint32_t)sample * 125 / 128
 *          - 当 sample=255 时，CCR = 255*125/128 = 249 (恰好等于 ARR)
 */
#ifndef HAL_PWM_HPP
#define HAL_PWM_HPP

#include "main.h"

namespace hal {

/**
 * @class Pwm
 * @brief PWM 输出抽象类
 * @details 封装 TIM PWM 的启停和占空比设置操作。
 *          配合高级定时器的 RepetitionCounter 功能，
 *          Update 中断仅在 RCR+1 次 PWM 周期后触发一次，
 *          实现"高频 PWM + 低频采样更新"的分离。
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
     * @details 由于 RCR=7，Update 中断每 8 个 PWM 周期触发一次，
     *          即频率 = 256kHz / 8 = 32kHz，恰好匹配音频采样率。
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
     *          当 Period=249 时，CCR=125 即 50% 占空比
     */
    void setCompare(uint32_t ccr) {
        __HAL_TIM_SET_COMPARE(htim_, channel_, ccr);
    }

    /**
     * @brief 将 8-bit 音频采样值转换为 CCR 并设置
     * @param sample 8-bit 无符号音频采样值 (0~255)
     * @details 映射公式：CCR = sample * (Period+1) / 256
     *          当 Period=249 时：CCR = sample * 250 / 256 = sample * 125 / 128
     *          验证：sample=0 → CCR=0, sample=255 → CCR=249 ✓
     */
    void setAudioSample(uint8_t sample) {
        // 将 8-bit 采样值映射到 0~249 的 CCR 范围
        // CCR = sample * 250 / 256 = sample * 125 / 128
        uint32_t ccr = static_cast<uint32_t>(sample) * 125UL / 128UL;
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
