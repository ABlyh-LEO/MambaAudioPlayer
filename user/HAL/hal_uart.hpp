/**
 * @file    hal_uart.hpp
 * @brief   UART 硬件抽象层封装
 * @details 对 STM32 HAL 库的 UART 操作进行封装，支持 DMA 收发和日志输出。
 *          USART1 配置：1Mbps, 8N1, DMA 收发。
 */
#ifndef HAL_UART_HPP
#define HAL_UART_HPP

#include "main.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace hal {

/**
 * @class Uart
 * @brief UART 通信抽象类
 * @details 封装 UART 的阻塞发送、DMA 接收和格式化日志输出。
 *          使用 DMA 空闲线路检测实现不定长数据接收。
 */
class Uart {
public:
    /// 发送缓冲区大小 (用于格式化日志)
    static constexpr uint16_t TX_BUF_SIZE = 256;

    /**
     * @brief 构造函数
     * @param huart UART 句柄指针（由 CubeMX 生成的 huart1 等）
     */
    explicit Uart(UART_HandleTypeDef* huart)
        : huart_(huart) {}

    /**
     * @brief 阻塞模式发送数据
     * @param data 发送数据缓冲区
     * @param size 数据长度
     * @param timeout 超时时间 (ms)
     * @return HAL_OK 成功
     */
    HAL_StatusTypeDef transmit(const uint8_t* data, uint16_t size, uint32_t timeout = 1000) {
        return HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), size, timeout);
    }

    /**
     * @brief DMA 模式发送数据 (非阻塞)
     * @param data 发送数据缓冲区
     * @param size 数据长度
     * @return HAL_OK 成功
     */
    HAL_StatusTypeDef transmitDMA(const uint8_t* data, uint16_t size) {
        return HAL_UART_Transmit_DMA(huart_, const_cast<uint8_t*>(data), size);
    }

    /**
     * @brief 启动 DMA 空闲中断接收
     * @param data 接收数据缓冲区
     * @param size 缓冲区最大长度
     * @return HAL_OK 成功
     * @details 使用 HAL_UARTEx_ReceiveToIdle_DMA 实现不定长接收，
     *          当总线空闲时自动触发回调。
     */
    HAL_StatusTypeDef receiveToIdleDMA(uint8_t* data, uint16_t size) {
        return HAL_UARTEx_ReceiveToIdle_DMA(huart_, data, size);
    }

    /**
     * @brief 格式化日志输出（printf 风格）
     * @param fmt 格式字符串
     * @param ... 可变参数
     * @details 使用阻塞模式发送，适用于调试日志。
     *          最大输出长度为 TX_BUF_SIZE 字节。
     */
    void log(const char* fmt, ...) {
        char buf[TX_BUF_SIZE];
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(buf, TX_BUF_SIZE, fmt, args);
        va_end(args);
        if (len > 0) {
            transmit(reinterpret_cast<const uint8_t*>(buf),
                     static_cast<uint16_t>(len), 500);
        }
    }

    /** @brief 获取底层 UART 句柄 */
    UART_HandleTypeDef* getHandle() const { return huart_; }

private:
    UART_HandleTypeDef* huart_;  ///< UART 句柄指针
};

} // namespace hal

#endif // HAL_UART_HPP
