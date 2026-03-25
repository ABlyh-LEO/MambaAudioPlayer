/**
 * @file    battery_monitor.hpp
 * @brief   电池状态综合监测模块
 * @details 整合 ADC 电压采集和 I2C 电池架数据，提供统一的电池状态查询接口。
 *
 *          电压采集公式：
 *          硬件分压 = 100kΩ / (100kΩ + 10kΩ) = 10/110 ≈ 1/11
 *          V_battery = V_adc * 11
 *          V_adc = ADC_raw * 3300mV / 4095
 *          V_battery(mV) = ADC_raw * 3300 * 11 / 4095
 *                        = ADC_raw * 36300 / 4095
 *
 *          低电量判定逻辑：
 *          - I2C 在线：以电池架报告的 capacity_percent 为准，< 35% 报警
 *          - I2C 离线：以 ADC 电压为准，< 21.5V (21500mV) 报警
 */
#ifndef BATTERY_MONITOR_HPP
#define BATTERY_MONITOR_HPP

#include "HAL/hal_adc.hpp"
#include "HDL/battery_rack.hpp"

namespace fml {

/**
 * @class BatteryMonitor
 * @brief 电池状态综合监测类
 * @details 周期性读取 ADC 和 I2C 数据，判断电池状态。
 */
class BatteryMonitor {
public:
    /// ADC 多次采样次数 (取平均值降低噪声)
    static constexpr uint8_t ADC_SAMPLE_COUNT = 16;

    /// I2C 低电量阈值 (百分比)
    static constexpr uint8_t LOW_BATTERY_PERCENT = 35;

    /// ADC 低电压阈值 (mV) - 24V 电池低于 21.5V
    static constexpr uint32_t LOW_VOLTAGE_MV = 21500;

    /// I2C 连接检测间隔 (ms)
    static constexpr uint32_t I2C_CHECK_INTERVAL_MS = 3000;

    /// ADC 采样间隔 (ms)
    static constexpr uint32_t ADC_SAMPLE_INTERVAL_MS = 1000;

    /**
     * @brief 构造函数
     * @param adc  ADC 抽象层引用
     * @param rack 电池架驱动引用
     */
    BatteryMonitor(hal::Adc& adc, hdl::BatteryRack& rack)
        : adc_(adc), rack_(rack),
          batteryVoltageMv_(0), i2cAvailable_(false),
          lastI2cCheckTick_(0), lastAdcTick_(0) {
        memset(&batteryInfo_, 0, sizeof(batteryInfo_));
    }

    /**
     * @brief 初始化电池监测
     * @details 执行 ADC 校准，首次检测 I2C 连接
     */
    void init() {
        adc_.calibrate();
        updateI2cConnection();
        updateAdcVoltage();
    }

    /**
     * @brief 周期性更新 (在主循环中调用)
     * @param tick 当前系统时间 (ms, HAL_GetTick)
     * @details 每隔固定时间更新 ADC 电压和 I2C 数据
     */
    void update(uint32_t tick) {
        // ADC 采样 (每 1 秒)
        if (tick - lastAdcTick_ >= ADC_SAMPLE_INTERVAL_MS) {
            lastAdcTick_ = tick;
            updateAdcVoltage();
        }

        // I2C 连接检测和数据读取 (每 3 秒)
        if (tick - lastI2cCheckTick_ >= I2C_CHECK_INTERVAL_MS) {
            lastI2cCheckTick_ = tick;
            updateI2cConnection();
        }
    }

    /**
     * @brief 获取 ADC 测量的电池电压 (mV)
     * @return 电池实际电压，单位 mV
     * @details 经过分压比 11:1 换算后的实际电池电压
     */
    uint32_t getVoltage() const { return batteryVoltageMv_; }

    /** @brief 获取 I2C 电池架数据 */
    const hdl::BatteryInfo& getBatteryInfo() const { return batteryInfo_; }

    /** @brief I2C 电池架是否在线 */
    bool isI2CAvailable() const { return i2cAvailable_; }

    /**
     * @brief 判断是否低电量
     * @return true 低电量需要报警; false 正常
     * @details 判定逻辑：
     *          - I2C 在线：capacity_percent < 35%
     *          - I2C 离线：batteryVoltageMv < 21500mV
     */
    bool isLowBattery() const {
        if (i2cAvailable_) {
            return batteryInfo_.capacity_percent < LOW_BATTERY_PERCENT;
        } else {
            return batteryVoltageMv_ < LOW_VOLTAGE_MV;
        }
    }

    /**
     * @brief 获取电池电量百分比
     * @return 如果 I2C 在线返回精确百分比; 否则根据 ADC 电压估算
     * @details ADC 估算公式 (24V 锂电池组, 6串):
     *          满电 ≈ 25.2V, 空电 ≈ 18V
     *          percent = (V - 18000) * 100 / (25200 - 18000)
     *                  = (V - 18000) * 100 / 7200
     */
    uint8_t getBatteryPercent() const {
        if (i2cAvailable_) {
            return batteryInfo_.capacity_percent;
        }
        // ADC 估算
        if (batteryVoltageMv_ >= 25200) return 100;
        if (batteryVoltageMv_ <= 18000) return 0;
        return static_cast<uint8_t>(
            (batteryVoltageMv_ - 18000) * 100 / 7200
        );
    }

private:
    hal::Adc& adc_;               ///< ADC 引用
    hdl::BatteryRack& rack_;      ///< 电池架驱动引用

    uint32_t batteryVoltageMv_;    ///< ADC 测量的电池电压 (mV)
    bool i2cAvailable_;            ///< I2C 电池架是否在线
    hdl::BatteryInfo batteryInfo_; ///< 电池架信息缓存

    uint32_t lastI2cCheckTick_;    ///< 上次 I2C 检测时间
    uint32_t lastAdcTick_;         ///< 上次 ADC 采样时间

    /** @brief 更新 ADC 电压测量 */
    void updateAdcVoltage() {
        // 多次采样取平均
        uint16_t rawAvg = adc_.readRawAverage(ADC_SAMPLE_COUNT);
        // V_battery(mV) = raw * 3300 * 11 / 4095
        // 分步计算避免 uint32 溢出: raw最大4095, 4095*36300=148,648,500 < 2^32
        batteryVoltageMv_ = static_cast<uint32_t>(rawAvg) * 36300UL / 4095UL;
    }

    /** @brief 更新 I2C 连接状态和数据 */
    void updateI2cConnection() {
        bool wasAvailable = i2cAvailable_;
        i2cAvailable_ = rack_.isConnected();
        
        if (i2cAvailable_) {
            // 如果刚连接上，或者为了保险，每次都读取静态数据 (静态数据也就 10 字节左右)
            rack_.readStaticData(batteryInfo_);
            rack_.readDynamicData(batteryInfo_);
        }
    }
};

} // namespace fml

#endif // BATTERY_MONITOR_HPP
