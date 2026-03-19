/**
 * @file    hal_spi.hpp
 * @brief   SPI 硬件抽象层封装
 * @details 对 STM32 HAL 库的 SPI 操作进行封装，支持轮询和 DMA 模式传输。
 *          SPI1 配置：Master, CPOL=Low, CPHA=1Edge, 8-bit, MSB First, 32Mbps
 */
#ifndef HAL_SPI_HPP
#define HAL_SPI_HPP

#include "main.h"

namespace hal {

/**
 * @class Spi
 * @brief SPI 传输抽象类
 * @details 封装 SPI 的发送、接收和全双工传输接口。
 *          注意：CS (片选) 信号由上层驱动自行管理。
 */
class Spi {
public:
    /**
     * @brief 构造函数
     * @param hspi SPI 句柄指针（由 CubeMX 生成的 hspi1 等）
     */
    explicit Spi(SPI_HandleTypeDef* hspi)
        : hspi_(hspi) {}

    /**
     * @brief 轮询模式发送数据
     * @param data 发送数据缓冲区
     * @param size 数据长度
     * @param timeout 超时时间 (ms)
     * @return HAL_OK 成功; 其他值失败
     */
    HAL_StatusTypeDef transmit(const uint8_t* data, uint16_t size, uint32_t timeout = 100) {
        return HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(data), size, timeout);
    }

    /**
     * @brief 轮询模式接收数据
     * @param data 接收数据缓冲区
     * @param size 数据长度
     * @param timeout 超时时间 (ms)
     * @return HAL_OK 成功; 其他值失败
     */
    HAL_StatusTypeDef receive(uint8_t* data, uint16_t size, uint32_t timeout = 100) {
        return HAL_SPI_Receive(hspi_, data, size, timeout);
    }

    /**
     * @brief 轮询模式全双工传输
     * @param txData 发送数据缓冲区
     * @param rxData 接收数据缓冲区
     * @param size   数据长度
     * @param timeout 超时时间 (ms)
     * @return HAL_OK 成功; 其他值失败
     */
    HAL_StatusTypeDef transmitReceive(const uint8_t* txData, uint8_t* rxData,
                                      uint16_t size, uint32_t timeout = 100) {
        return HAL_SPI_TransmitReceive(hspi_, const_cast<uint8_t*>(txData), rxData, size, timeout);
    }

    /**
     * @brief DMA 模式接收数据 (非阻塞)
     * @param data 接收数据缓冲区
     * @param size 数据长度
     * @return HAL_OK 成功; 其他值失败
     */
    HAL_StatusTypeDef receiveDMA(uint8_t* data, uint16_t size) {
        return HAL_SPI_Receive_DMA(hspi_, data, size);
    }

    /**
     * @brief DMA 模式发送数据 (非阻塞)
     * @param data 发送数据缓冲区
     * @param size 数据长度
     * @return HAL_OK 成功; 其他值失败
     */
    HAL_StatusTypeDef transmitDMA(const uint8_t* data, uint16_t size) {
        return HAL_SPI_Transmit_DMA(hspi_, const_cast<uint8_t*>(data), size);
    }

    /** @brief 获取底层 SPI 句柄（供回调中识别实例用） */
    SPI_HandleTypeDef* getHandle() const { return hspi_; }

private:
    SPI_HandleTypeDef* hspi_;  ///< SPI 句柄指针
};

} // namespace hal

#endif // HAL_SPI_HPP
