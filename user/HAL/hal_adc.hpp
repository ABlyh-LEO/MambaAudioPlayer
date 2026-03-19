/**
 * @file    hal_adc.hpp
 * @brief   ADC 硬件抽象层封装
 * @details 对 STM32 HAL 库的 ADC 操作进行封装，提供电压采集接口。
 *          ADC 配置：12-bit 分辨率，软件触发，单次转换模式。
 */
#ifndef HAL_ADC_HPP
#define HAL_ADC_HPP

#include "main.h"

namespace hal {

/**
 * @class Adc
 * @brief ADC 采样抽象类
 * @details 封装 ADC 的校准、启动转换和读取操作。
 *          转换公式详见各方法注释。
 */
class Adc {
public:
    /**
     * @brief 构造函数
     * @param hadc ADC 句柄指针（由 CubeMX 生成的 hadc1 等）
     */
    explicit Adc(ADC_HandleTypeDef* hadc)
        : hadc_(hadc) {}

    /**
     * @brief ADC 校准（上电后调用一次）
     * @details STM32G0 ADC 需要在使用前进行校准以获得精确结果
     */
    void calibrate() {
        HAL_ADCEx_Calibration_Start(hadc_);
    }

    /**
     * @brief 执行单次 ADC 转换并返回原始值
     * @return 12-bit ADC 原始值 (0~4095)
     * @details 启动转换 → 等待完成 → 读取结果
     *          超时时间 100ms，超时返回 0
     */
    uint16_t readRaw() {
        HAL_ADC_Start(hadc_);
        if (HAL_ADC_PollForConversion(hadc_, 100) == HAL_OK) {
            uint16_t val = static_cast<uint16_t>(HAL_ADC_GetValue(hadc_));
            HAL_ADC_Stop(hadc_);
            return val;
        }
        HAL_ADC_Stop(hadc_);
        return 0;
    }

    /**
     * @brief 读取 ADC 引脚电压（单位：mV）
     * @return ADC 引脚实际电压，单位 mV
     * @details 计算公式：V_adc = raw * VREF / ADC_MAX
     *          - VREF = 3300mV (STM32 内部参考电压)
     *          - ADC_MAX = 4095 (12-bit ADC 最大值)
     *          - V_adc(mV) = raw * 3300 / 4095
     */
    uint32_t readVoltageMv() {
        uint16_t raw = readRaw();
        // V_adc(mV) = raw * 3300 / 4095
        return static_cast<uint32_t>(raw) * 3300UL / 4095UL;
    }

    /**
     * @brief 多次采样取平均值（滤波）
     * @param samples 采样次数
     * @return 平均后的 12-bit 原始值
     */
    uint16_t readRawAverage(uint8_t samples) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < samples; ++i) {
            sum += readRaw();
        }
        return static_cast<uint16_t>(sum / samples);
    }

private:
    ADC_HandleTypeDef* hadc_;  ///< ADC 句柄指针
};

} // namespace hal

#endif // HAL_ADC_HPP
