/**
 * @file    ws2812_driver.hpp
 * @brief   WS2812 RGB LED 驱动
 * @details 通过 GPIO bit-bang 实现 WS2812 单线协议。
 *          WS2812 时序协议 (800KHz):
 *          ┌────┐         ┌──────────┐
 *          │T0H │  T0L    │   T1H    │  T1L
 *          │0.4μs│ 0.85μs │  0.8μs   │ 0.45μs
 *          └────┘         └──────────┘
 *
 *          @64MHz 系统时钟，每个时钟周期 = 15.625ns
 *          T0H = 0.4μs ≈ 26 cycles
 *          T0L = 0.85μs ≈ 54 cycles
 *          T1H = 0.8μs ≈ 51 cycles
 *          T1L = 0.45μs ≈ 29 cycles
 *          Reset ≥ 50μs ≈ 3200 cycles
 *
 *          数据发送顺序：GRB (高位在前)
 *
 *          亮度限制：最大 50% (128/255)，避免电流过大对电路造成压力。
 *          单颗 WS2812 满亮度约 60mA，限制后约 30mA。
 */
#ifndef WS2812_DRIVER_HPP
#define WS2812_DRIVER_HPP

#include "main.h"

namespace hdl {

/**
 * @class WS2812Driver
 * @brief WS2812 RGB LED 控制驱动类 (单颗灯珠)
 * @details 使用 GPIO 直接控制 WS2812 的数据线。
 *          由于时序要求极为严格 (纳秒级), 发送过程中必须关闭中断。
 *          使用 NOP 指令实现精确延时。
 */
class WS2812Driver {
public:
    /// 最大亮度比例 (0~255, 128 = 50%) - 限制功率消耗
    static constexpr uint8_t MAX_BRIGHTNESS = 128;

    /**
     * @brief 构造函数
     * @param port GPIO 端口 (如 GPIOA)
     * @param pin  GPIO 引脚编号 (如 GPIO_PIN_5)
     */
    WS2812Driver(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    /**
     * @brief 设置 LED 颜色
     * @param r 红色分量 (0~255)
     * @param g 绿色分量 (0~255)
     * @param b 蓝色分量 (0~255)
     * @details 颜色值会被亮度限幅后发送给 WS2812。
     *          亮度限制公式：actual = color * MAX_BRIGHTNESS / 255
     *          防止过大电流损坏电路。
     */
    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        // 亮度限幅：将颜色值按比例缩放到最大亮度
        r = static_cast<uint8_t>(static_cast<uint16_t>(r) * MAX_BRIGHTNESS / 255);
        g = static_cast<uint8_t>(static_cast<uint16_t>(g) * MAX_BRIGHTNESS / 255);
        b = static_cast<uint8_t>(static_cast<uint16_t>(b) * MAX_BRIGHTNESS / 255);

        // WS2812 数据发送顺序：G -> R -> B (高位在前)
        __disable_irq();  // 关闭中断保证时序精度
        sendByte(g);
        sendByte(r);
        sendByte(b);
        __enable_irq();   // 恢复中断

        // 发送 Reset 信号 (数据线低电平保持 >50μs)
        resetSignal();
    }

    /**
     * @brief 熄灭 LED
     * @details 发送全零数据 (黑色)
     */
    void off() {
        setColor(0, 0, 0);
    }

private:
    GPIO_TypeDef* port_;  ///< GPIO 端口
    uint16_t pin_;        ///< GPIO 引脚

    /**
     * @brief 发送一个字节 (8 bit, MSB first)
     * @param byte 要发送的字节值
     * @details 逐位发送，根据位值选择 T0 或 T1 时序。
     *          使用直接寄存器操作和 NOP 延时保证纳秒级时序精度。
     */
    void sendByte(uint8_t byte) {
        for (int8_t i = 7; i >= 0; --i) {
            if (byte & (1 << i)) {
                sendBit1();
            } else {
                sendBit0();
            }
        }
    }

    /**
     * @brief 发送逻辑 "0"
     * @details T0H ≈ 0.4μs (26 cycles @64MHz)
     *          T0L ≈ 0.85μs (54 cycles @64MHz)
     *          使用 BSRR 寄存器直接操作，比 HAL 函数快得多
     */
    inline void sendBit0() {
        // 拉高数据线 (T0H)
        port_->BSRR = pin_;
        // 延时约 0.35~0.4μs (约 22~26 个 NOP)
        nopDelay(20);
        // 拉低数据线 (T0L)
        port_->BRR = pin_;
        // 延时约 0.8~0.85μs (约 50~54 个 NOP)
        nopDelay(46);
    }

    /**
     * @brief 发送逻辑 "1"
     * @details T1H ≈ 0.8μs (51 cycles @64MHz)
     *          T1L ≈ 0.45μs (29 cycles @64MHz)
     */
    inline void sendBit1() {
        // 拉高数据线 (T1H)
        port_->BSRR = pin_;
        // 延时约 0.7~0.8μs (约 45~51 个 NOP)
        nopDelay(45);
        // 拉低数据线 (T1L)
        port_->BRR = pin_;
        // 延时约 0.4~0.45μs (约 25~29 个 NOP)
        nopDelay(22);
    }

    /**
     * @brief NOP 延时
     * @param count NOP 指令数量
     * @details 每个 NOP 在 Cortex-M0+ @64MHz 上约 15.625ns
     *          使用内联汇编确保编译器不会优化掉
     */
    static inline void nopDelay(uint32_t count) {
        for (volatile uint32_t i = 0; i < count; ++i) {
            __NOP();
        }
    }

    /**
     * @brief 发送 Reset 信号
     * @details 数据线保持低电平超过 50μs
     *          使用 HAL_Delay 足够（精度 1ms，远超 50μs 要求）
     */
    void resetSignal() {
        port_->BRR = pin_;  // 拉低
        // 延迟约 80μs
        for (volatile uint32_t i = 0; i < 400; ++i) {
            __NOP();
        }
    }
};

} // namespace hdl

#endif // WS2812_DRIVER_HPP
