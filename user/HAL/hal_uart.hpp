/**
 * @file    hal_uart.hpp
 * @brief   UART 硬件抽象层封装
 * @details 对 STM32 HAL 库的 UART 操作进行封装，支持 DMA 收发和日志输出。
 *          USART1 配置：1Mbps, 8N1, DMA 收发。
 */
#ifndef HAL_UART_HPP
#define HAL_UART_HPP

#include "main.h"
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
     * @brief 启动 DMA 接收 (定长接收)
     * @param data 接收数据缓冲区
     * @param size 要接收的精确字节数
     * @return HAL_OK 成功
     * @details 使用 HAL_UART_Receive_DMA 接收定长数据，不受 USB 帧间微小 IDLE 间隙影响。
     */
    HAL_StatusTypeDef receiveDMA(uint8_t* data, uint16_t size) {
        HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart_, data, size);
        if (status == HAL_OK) {
            // 禁用 DMA 半传输中断，要求必须收到完整一帧
            __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
        }
        return status;
    }

    /**
     * @brief 格式化日志输出（printf 风格）- ⚠️ 已禁用以节省 Flash
     * @param fmt 格式字符串
     * @param ... 可变参数
     * @details 为缩小编译体积 (节省近 5KB Flash)，日志输出功能已被空实现。
     *          当前状态查询可通过 QUERY_STATUS 命令从上位机获取。
     *          如有调试需要，可还原内部 vsnprintf 实现。
     */
    void log(const char* fmt, ...) {
        (void)fmt;
        // 已注释以节省 Flash
        // char buf[TX_BUF_SIZE];
        // va_list args;
        // va_start(args, fmt);
        // int len = vsnprintf(buf, TX_BUF_SIZE, fmt, args);
        // va_end(args);
        // if (len > 0) {
        //     transmit(reinterpret_cast<const uint8_t*>(buf), static_cast<uint16_t>(len), 500);
        // }
    }

    /**
     * @brief 清除 UART 错误标志并重置接收状态
     * @details 清除 ORE/FE/NE/PE 等错误标志，强制中止当前接收并重置状态机。
     *          用于从 Overrun 等硬件错误中恢复。
     */
    void clearErrors() {
        // 清除错误标志
        __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_OREF | UART_CLEAR_FEF
                                      | UART_CLEAR_NEF | UART_CLEAR_PEF);
        
        // 即使状态是 READY，如果 ErrorCode 还在，也要重置
        if (huart_->RxState != HAL_UART_STATE_READY || huart_->ErrorCode != HAL_UART_ERROR_NONE) {
            HAL_UART_AbortReceive(huart_);
        }
    }

    /**
     * @brief 检查 UART 接收是否正在进行 (DMA)
     * @return true 正在接收; false 空闲
     */
    bool isBusyRx() const {
        return huart_->RxState != HAL_UART_STATE_READY;
    }

    /** @brief 获取底层 UART 句柄 */
    UART_HandleTypeDef* getHandle() const { return huart_; }

private:
    UART_HandleTypeDef* huart_;  ///< UART 句柄指针
};

} // namespace hal

#endif // HAL_UART_HPP
