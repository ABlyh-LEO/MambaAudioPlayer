/**
 * @file    battery_rack.hpp
 * @brief   电池架 I2C 通信驱动
 * @details 通过 I2C 总线读取电池架的各种状态信息。
 *
 *          电池架 I2C 参数：
 *          - 从机地址：0x41 (7-bit)
 *          - 通信速率：100kHz
 *          - 需要外部 4.7kΩ 上拉电阻
 *
 *          寄存器映射：
 *          静态数据 (更新间隔 3s):
 *            0x1F  WHO_AM_I         uint8_t   固定值 0x41
 *            0x20  DESIGNED_CAPACITY uint32_t  设计容量 (mAh)
 *            0x24  LOOP_TIMES       uint16_t  充放电循环次数
 *            0x26  PRODUCTION_DATE  uint16_t  生产日期
 *            0x28  BATTERY_LIFE     uint8_t   寿命百分比 (0~100%)
 *          动态数据 (更新间隔 500ms):
 *            0x29  CURRENT_VOLTAGE  int32_t   当前电压 (mV)
 *            0x2D  CURRENT_CURRENT  int32_t   当前电流 (mA)
 *            0x31  TEMPERATURE      int16_t   温度 (0.1°C精度)
 *            0x33  CAPACITY_PRECENT uint8_t   容量百分比 (0~100%)
 *            0x34  INTERNAL_STATE   uint8_t   内部状态
 *            0x35  ERROR_STATE      uint8_t   错误状态
 */
#ifndef BATTERY_RACK_HPP
#define BATTERY_RACK_HPP

#include "HAL/hal_i2c.hpp"

namespace hdl {

/// 电池架寄存器地址定义
namespace BattReg {
    // 静态数据寄存器
    constexpr uint8_t WHO_AM_I          = 0x1F;  ///< 识别码 (固定 0x41)
    constexpr uint8_t DESIGNED_CAPACITY = 0x20;  ///< 设计容量 (4 bytes, mAh)
    constexpr uint8_t LOOP_TIMES        = 0x24;  ///< 充放电循环次数 (2 bytes)
    constexpr uint8_t PRODUCTION_DATE   = 0x26;  ///< 生产日期 (2 bytes)
    constexpr uint8_t BATTERY_LIFE      = 0x28;  ///< 寿命百分比 (1 byte)

    // 动态数据寄存器
    constexpr uint8_t CURRENT_VOLTAGE   = 0x29;  ///< 当前电压 (4 bytes, mV)
    constexpr uint8_t CURRENT_CURRENT   = 0x2D;  ///< 当前电流 (4 bytes, mA)
    constexpr uint8_t TEMPERATURE       = 0x31;  ///< 温度 (2 bytes, 0.1°C)
    constexpr uint8_t CAPACITY_PERCENT  = 0x33;  ///< 容量百分比 (1 byte, 0~100%)
    constexpr uint8_t INTERNAL_STATE    = 0x34;  ///< 内部状态 (1 byte)
    constexpr uint8_t ERROR_STATE       = 0x35;  ///< 错误状态 (1 byte)
}

/// 电池架从机地址 (7-bit)
constexpr uint8_t BATTERY_RACK_ADDR = 0x41;

/**
 * @struct BatteryInfo
 * @brief 电池信息综合数据结构
 * @details 存储从电池架读取的所有信息
 */
struct BatteryInfo {
    // 静态数据
    uint32_t designed_capacity;  ///< 设计容量 (mAh)
    uint16_t loop_times;         ///< 充放电循环次数
    uint16_t production_date;    ///< 生产日期 (原始值)
    uint8_t  battery_life;       ///< 寿命百分比 (0~100%)

    // 动态数据
    int32_t  voltage_mv;         ///< 当前电压 (mV)
    int32_t  current_ma;         ///< 当前电流 (mA)
    int16_t  temperature;        ///< 温度 (0.1°C 精度)
    uint8_t  capacity_percent;   ///< 容量百分比 (0~100%)
    uint8_t  internal_state;     ///< 内部状态原始值
    uint8_t  error_state;        ///< 错误状态原始值
};

/**
 * @struct ProductionDate
 * @brief 生产日期解析结构
 * @details 从 PRODUCTION_DATE 寄存器的 uint16_t 值解析：
 *          bit[15:9] = 1980 到今年的年数
 *          bit[8:5]  = 月
 *          bit[4:0]  = 日
 */
struct ProductionDate {
    uint16_t year;   ///< 实际年份 (如 2024)
    uint8_t  month;  ///< 月份 (1~12)
    uint8_t  day;    ///< 日 (1~31)
};

/**
 * @struct InternalState
 * @brief 内部状态位域解析
 * @details INTERNAL_STATE 寄存器:
 *          bit[1] = 与电池的连接状态 (0=异常, 1=正常)
 *          bit[0] = 电池输出状态 (0=小电流≤0.2A, 1=大电流>0.2A)
 */
struct InternalState {
    bool battery_connected;  ///< 与电池通信是否正常
    bool high_current;       ///< 是否处于大电流状态
};

/**
 * @struct ErrorState
 * @brief 错误状态位域解析
 * @details ERROR_STATE 寄存器:
 *          bit[6] = 自检异常
 *          bit[5] = 电芯异常
 *          bit[4] = 电池欠压
 *          bit[3] = 电池过温
 *          bit[2] = 电池过流
 *          bit[1] = 电池放电过载
 *          bit[0] = 电池放电短路
 */
struct ErrorState {
    bool self_check_error;   ///< 自检异常
    bool cell_error;         ///< 电芯异常
    bool under_voltage;      ///< 电池欠压
    bool over_temperature;   ///< 电池过温
    bool over_current;       ///< 电池过流
    bool discharge_overload; ///< 放电过载
    bool discharge_short;    ///< 放电短路
};

/**
 * @class BatteryRack
 * @brief 电池架 I2C 通信驱动类
 * @details 通过 I2C 总线与电池架通信，读取电池信息。
 */
class BatteryRack {
public:
    /**
     * @brief 构造函数
     * @param i2c I2C 抽象层对象引用
     */
    explicit BatteryRack(hal::I2c& i2c)
        : i2c_(i2c), connected_(false) {}

    /**
     * @brief 检测电池架是否在线
     * @return true 在线; false 离线
     * @details 通过读取 WHO_AM_I 寄存器判断，期望值 0x41
     */
    bool isConnected() {
        uint8_t who = 0;
        if (i2c_.memRead(BATTERY_RACK_ADDR, BattReg::WHO_AM_I, &who, 1, 50) == HAL_OK) {
            connected_ = (who == BATTERY_RACK_ADDR);
        } else {
            connected_ = false;
        }
        return connected_;
    }

    /**
     * @brief 读取所有动态数据
     * @param info 输出电池信息结构体
     * @return true 读取成功; false 读取失败
     */
    bool readDynamicData(BatteryInfo& info) {
        bool ok = true;
        ok &= readRegister(BattReg::CURRENT_VOLTAGE,
                           reinterpret_cast<uint8_t*>(&info.voltage_mv), 4);
        ok &= readRegister(BattReg::CURRENT_CURRENT,
                           reinterpret_cast<uint8_t*>(&info.current_ma), 4);
        ok &= readRegister(BattReg::TEMPERATURE,
                           reinterpret_cast<uint8_t*>(&info.temperature), 2);
        ok &= readRegister(BattReg::CAPACITY_PERCENT,
                           &info.capacity_percent, 1);
        ok &= readRegister(BattReg::INTERNAL_STATE,
                           &info.internal_state, 1);
        ok &= readRegister(BattReg::ERROR_STATE,
                           &info.error_state, 1);
        return ok;
    }

    /**
     * @brief 读取所有静态数据
     * @param info 输出电池信息结构体
     * @return true 读取成功; false 读取失败
     */
    bool readStaticData(BatteryInfo& info) {
        bool ok = true;
        ok &= readRegister(BattReg::DESIGNED_CAPACITY,
                           reinterpret_cast<uint8_t*>(&info.designed_capacity), 4);
        ok &= readRegister(BattReg::LOOP_TIMES,
                           reinterpret_cast<uint8_t*>(&info.loop_times), 2);
        ok &= readRegister(BattReg::PRODUCTION_DATE,
                           reinterpret_cast<uint8_t*>(&info.production_date), 2);
        ok &= readRegister(BattReg::BATTERY_LIFE,
                           &info.battery_life, 1);
        return ok;
    }

    /**
     * @brief 解析生产日期
     * @param rawDate PRODUCTION_DATE 寄存器原始值
     * @return 解析后的日期结构体
     * @details 位域解析：
     *          year  = bit[15:9] + 1980
     *          month = bit[8:5]
     *          day   = bit[4:0]
     */
    static ProductionDate parseProductionDate(uint16_t rawDate) {
        ProductionDate date;
        date.year  = ((rawDate >> 9) & 0x7F) + 1980;
        date.month = (rawDate >> 5) & 0x0F;
        date.day   = rawDate & 0x1F;
        return date;
    }

    /**
     * @brief 解析内部状态寄存器
     * @param raw INTERNAL_STATE 寄存器原始值
     * @return 解析后的内部状态结构体
     * @details bit[1] = 与电池连接状态 (0=异常, 1=正常)
     *          bit[0] = 输出状态 (0=小电流≤0.2A会自动关闭, 1=大电流>0.2A)
     */
    static InternalState parseInternalState(uint8_t raw) {
        InternalState state;
        state.battery_connected = (raw >> 1) & 0x01;
        state.high_current      = raw & 0x01;
        return state;
    }

    /**
     * @brief 解析错误状态寄存器
     * @param raw ERROR_STATE 寄存器原始值
     * @return 解析后的错误状态结构体
     */
    static ErrorState parseErrorState(uint8_t raw) {
        ErrorState err;
        err.self_check_error   = (raw >> 6) & 0x01;
        err.cell_error         = (raw >> 5) & 0x01;
        err.under_voltage      = (raw >> 4) & 0x01;
        err.over_temperature   = (raw >> 3) & 0x01;
        err.over_current       = (raw >> 2) & 0x01;
        err.discharge_overload = (raw >> 1) & 0x01;
        err.discharge_short    = raw & 0x01;
        return err;
    }

    /** @brief 获取最近一次连接状态 */
    bool lastConnected() const { return connected_; }

private:
    hal::I2c& i2c_;    ///< I2C 抽象层引用
    bool connected_;   ///< 连接状态缓存

    /**
     * @brief 读取寄存器通用方法
     * @param regAddr 寄存器地址
     * @param data    数据缓冲区
     * @param size    数据长度
     * @return true 成功; false 失败
     */
    bool readRegister(uint8_t regAddr, uint8_t* data, uint16_t size) {
        return i2c_.memRead(BATTERY_RACK_ADDR, regAddr, data, size, 100) == HAL_OK;
    }
};

} // namespace hdl

#endif // BATTERY_RACK_HPP
