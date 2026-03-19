/**
 * @file    hal_gpio.hpp
 * @brief   GPIO 硬件抽象层封装
 * @details 对 STM32 HAL 库的 GPIO 操作进行面向对象封装，
 *          提供统一的引脚读写接口。
 */
#ifndef HAL_GPIO_HPP
#define HAL_GPIO_HPP

#include "main.h"

namespace hal {

/**
 * @class Gpio
 * @brief GPIO 引脚抽象类
 * @details 封装单个 GPIO 引脚的读写操作，通过构造时传入 Port 和 Pin 参数绑定硬件引脚。
 */
class Gpio {
public:
    /**
     * @brief 构造函数
     * @param port GPIO 端口指针 (如 GPIOA, GPIOB, GPIOC)
     * @param pin  GPIO 引脚编号 (如 GPIO_PIN_0, GPIO_PIN_7)
     */
    Gpio(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    /** @brief 将引脚设置为高电平 */
    void setHigh() {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    }

    /** @brief 将引脚设置为低电平 */
    void setLow() {
        HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
    }

    /**
     * @brief 设置引脚电平
     * @param state true=高电平, false=低电平
     */
    void write(bool state) {
        HAL_GPIO_WritePin(port_, pin_, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /** @brief 翻转引脚电平 */
    void toggle() {
        HAL_GPIO_TogglePin(port_, pin_);
    }

    /**
     * @brief 读取引脚电平
     * @return true=高电平, false=低电平
     */
    bool read() const {
        return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    }

private:
    GPIO_TypeDef* port_;  ///< GPIO 端口指针
    uint16_t pin_;        ///< GPIO 引脚编号
};

} // namespace hal

#endif // HAL_GPIO_HPP
