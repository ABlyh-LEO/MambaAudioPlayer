/**
 * @file    hal_i2c.hpp
 * @brief   I2C 硬件抽象层封装
 * @details 对 STM32 HAL 库的 I2C 主机操作进行封装。
 *          I2C2 配置：100kHz, 7-bit 地址模式。
 */
#ifndef HAL_I2C_HPP
#define HAL_I2C_HPP

#include "main.h"

namespace hal {

/**
 * @class I2c
 * @brief I2C 主机通信抽象类
 * @details 封装 I2C 的寄存器读写操作。
 *          注意：HAL 库中地址参数为 7-bit 地址左移1位。
 */
class I2c {
public:
    /**
     * @brief 构造函数
     * @param hi2c I2C 句柄指针（由 CubeMX 生成的 hi2c2 等）
     */
    explicit I2c(I2C_HandleTypeDef* hi2c)
        : hi2c_(hi2c) {}

    /**
     * @brief 从指定设备的指定寄存器读取数据
     * @param devAddr 设备 7-bit 地址 (未左移，如 0x41)
     * @param regAddr 寄存器地址
     * @param data    接收数据缓冲区
     * @param size    读取数据长度 (字节)
     * @param timeout 超时时间 (ms)
     * @return HAL_OK 成功; HAL_ERROR/HAL_TIMEOUT 失败
     * @details HAL 库要求地址左移1位，此处自动处理：devAddr << 1
     */
    HAL_StatusTypeDef memRead(uint8_t devAddr, uint8_t regAddr,
                              uint8_t* data, uint16_t size, uint32_t timeout = 100) {
        // HAL_I2C_Mem_Read 的地址参数需要左移1位
        return HAL_I2C_Mem_Read(hi2c_, static_cast<uint16_t>(devAddr) << 1,
                                regAddr, I2C_MEMADD_SIZE_8BIT,
                                data, size, timeout);
    }

    /**
     * @brief 向指定设备的指定寄存器写入数据
     * @param devAddr 设备 7-bit 地址 (未左移)
     * @param regAddr 寄存器地址
     * @param data    发送数据缓冲区
     * @param size    写入数据长度 (字节)
     * @param timeout 超时时间 (ms)
     * @return HAL_OK 成功; HAL_ERROR/HAL_TIMEOUT 失败
     */
    HAL_StatusTypeDef memWrite(uint8_t devAddr, uint8_t regAddr,
                               const uint8_t* data, uint16_t size, uint32_t timeout = 100) {
        return HAL_I2C_Mem_Write(hi2c_, static_cast<uint16_t>(devAddr) << 1,
                                 regAddr, I2C_MEMADD_SIZE_8BIT,
                                 const_cast<uint8_t*>(data), size, timeout);
    }

    /**
     * @brief 检测设备是否在线
     * @param devAddr 设备 7-bit 地址 (未左移)
     * @param trials  重试次数
     * @param timeout 每次尝试超时时间 (ms)
     * @return true 设备在线; false 设备离线
     */
    bool isDeviceReady(uint8_t devAddr, uint8_t trials = 3, uint32_t timeout = 50) {
        return HAL_I2C_IsDeviceReady(hi2c_, static_cast<uint16_t>(devAddr) << 1,
                                      trials, timeout) == HAL_OK;
    }

private:
    I2C_HandleTypeDef* hi2c_;  ///< I2C 句柄指针
};

} // namespace hal

#endif // HAL_I2C_HPP
