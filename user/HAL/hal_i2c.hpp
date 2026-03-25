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
     * @details 增加重试机制和总线恢复处理
     */
    HAL_StatusTypeDef memRead(uint8_t devAddr, uint8_t regAddr,
                              uint8_t* data, uint16_t size, uint32_t timeout = 100) {
        HAL_StatusTypeDef status;
        for (uint8_t i = 0; i < 3; ++i) {
            status = HAL_I2C_Mem_Read(hi2c_, static_cast<uint16_t>(devAddr) << 1,
                                      regAddr, I2C_MEMADD_SIZE_8BIT,
                                      data, size, timeout);
            if (status == HAL_OK) return HAL_OK;
            
            // 发生错误，尝试恢复总线
            recover();
            HAL_Delay(5); 
        }
        return status;
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
        HAL_StatusTypeDef status;
        for (uint8_t i = 0; i < 3; ++i) {
            status = HAL_I2C_Mem_Write(hi2c_, static_cast<uint16_t>(devAddr) << 1,
                                       regAddr, I2C_MEMADD_SIZE_8BIT,
                                       const_cast<uint8_t*>(data), size, timeout);
            if (status == HAL_OK) return HAL_OK;
            
            recover();
            HAL_Delay(5);
        }
        return status;
    }

    /**
     * @brief 检测设备是否在线
     */
    bool isDeviceReady(uint8_t devAddr, uint8_t trials = 3, uint32_t timeout = 50) {
        if (HAL_I2C_IsDeviceReady(hi2c_, static_cast<uint16_t>(devAddr) << 1, trials, timeout) == HAL_OK) {
            return true;
        }
        // 如果不在线，尝试恢复一次后再测
        recover();
        return HAL_I2C_IsDeviceReady(hi2c_, static_cast<uint16_t>(devAddr) << 1, 1, timeout) == HAL_OK;
    }

    /**
     * @brief I2C 错误恢复
     * @details 尝试清除 busy 标志并重新初始化 I2C 外设
     */
    void recover() {
        // 如果发生错误或处于 Busy 状态，重置 I2C
        // 注意：底层 HAL_I2C_Init 会处理大部分重置逻辑
        HAL_I2C_DeInit(hi2c_);
        HAL_Delay(1);
        HAL_I2C_Init(hi2c_);
    }

private:
    I2C_HandleTypeDef* hi2c_;  ///< I2C 句柄指针
};

} // namespace hal

#endif // HAL_I2C_HPP
