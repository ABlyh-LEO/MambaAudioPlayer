/**
 * @file    w25q64_driver.hpp
 * @brief   W25Q64 SPI Flash 驱动 (8MB NOR Flash)
 * @details 基于 SPI 硬件抽象层实现 W25Q64JV 的读写擦除操作。
 *          支持 DMA 读取以加速音频数据流式读取。
 *
 *          W25Q64 参数：
 *          - 容量：64Mbit = 8MByte
 *          - 页大小：256 bytes
 *          - 扇区大小：4096 bytes (4KB)
 *          - 块大小：64KB (16个扇区)
 *          - 共 128 个块，2048 个扇区
 *
 *          Flash 数据存储布局设计：
 *          ┌─────────────────────────────────────────────────┐
 *          │ 偏移量 0x000000 - 0x000FFF (Sector 0, 4KB)     │
 *          │ ┌─────────────────────────────────────────────┐ │
 *          │ │ 音频索引表头部 (16 bytes):                  │ │
 *          │ │   magic[4]    = 0x4D414D42 ("MAMB")         │ │
 *          │ │   count[1]    = 音频段数 (最大 32)          │ │
 *          │ │   reserved[11]= 保留                        │ │
 *          │ ├─────────────────────────────────────────────┤ │
 *          │ │ 音频索引条目 entry[0..31] (每个16 bytes):   │ │
 *          │ │   start_addr[4] = 音频数据起始地址          │ │
 *          │ │   length[4]     = 音频数据长度 (bytes)      │ │
 *          │ │   sample_rate[2]= 采样率 (Hz, 如 16000)     │ │
 *          │ │   bits[1]       = 位深 (8)                  │ │
 *          │ │   reserved[5]   = 保留                      │ │
 *          │ └─────────────────────────────────────────────┘ │
 *          ├─────────────────────────────────────────────────┤
 *          │ 偏移量 0x001000 - 0x7FFFFF (~8MB)              │
 *          │   音频数据区 (8-bit unsigned PCM)               │
 *          └─────────────────────────────────────────────────┘
 */
#ifndef W25Q64_DRIVER_HPP
#define W25Q64_DRIVER_HPP

#include "HAL/hal_spi.hpp"
#include "HAL/hal_gpio.hpp"

namespace hdl {

/// W25Q64 SPI Flash 指令集
namespace W25Q64Cmd {
    constexpr uint8_t WRITE_ENABLE       = 0x06;  ///< 写使能
    constexpr uint8_t WRITE_DISABLE      = 0x04;  ///< 写禁止
    constexpr uint8_t READ_STATUS_REG1   = 0x05;  ///< 读状态寄存器1
    constexpr uint8_t READ_DATA          = 0x03;  ///< 读取数据 (最高 50MHz)
    constexpr uint8_t FAST_READ          = 0x0B;  ///< 快速读取 (最高 133MHz)
    constexpr uint8_t PAGE_PROGRAM       = 0x02;  ///< 页编程 (256 bytes)
    constexpr uint8_t SECTOR_ERASE       = 0x20;  ///< 扇区擦除 (4KB)
    constexpr uint8_t BLOCK_ERASE_32K    = 0x52;  ///< 块擦除 (32KB)
    constexpr uint8_t BLOCK_ERASE_64K    = 0xD8;  ///< 块擦除 (64KB)
    constexpr uint8_t CHIP_ERASE         = 0xC7;  ///< 全片擦除
    constexpr uint8_t READ_JEDEC_ID      = 0x9F;  ///< 读取 JEDEC ID
    constexpr uint8_t POWER_DOWN         = 0xB9;  ///< 掉电模式
    constexpr uint8_t RELEASE_POWER_DOWN = 0xAB;  ///< 释放掉电模式
}

/// 音频索引表常量
namespace AudioIndex {
    constexpr uint32_t MAGIC            = 0x4D414D42;  ///< "MAMB" 魔数
    constexpr uint32_t INDEX_BASE_ADDR  = 0x000000;    ///< 索引表起始地址
    constexpr uint32_t AUDIO_DATA_BASE  = 0x001000;    ///< 音频数据区起始地址 (Sector 1)
    constexpr uint8_t  MAX_TRACKS       = 32;          ///< 最大支持音频段数
    constexpr uint16_t HEADER_SIZE      = 16;          ///< 索引表头部大小
    constexpr uint16_t ENTRY_SIZE       = 16;          ///< 每条索引条目大小
}

/// W25Q64 参数常量
namespace W25Q64Param {
    constexpr uint32_t PAGE_SIZE        = 256;         ///< 页大小 (bytes)
    constexpr uint32_t SECTOR_SIZE      = 4096;        ///< 扇区大小 (bytes)
    constexpr uint32_t BLOCK_SIZE       = 65536;       ///< 块大小 (bytes, 64KB)
    constexpr uint32_t TOTAL_SIZE       = 8 * 1024 * 1024;  ///< 总容量 (8MB)
    constexpr uint8_t  STATUS_BUSY_BIT  = 0x01;        ///< 状态寄存器 BUSY 位
}

/// 音频索引表头部结构
#pragma pack(push, 1)
struct AudioIndexHeader {
    uint32_t magic;         ///< 魔数 0x4D414D42
    uint8_t  count;         ///< 音频段数
    uint8_t  reserved[11];  ///< 保留字段
};

/// 单条音频索引条目
struct AudioIndexEntry {
    uint32_t start_addr;    ///< 音频数据起始地址
    uint32_t length;        ///< 音频数据长度 (bytes)
    uint16_t sample_rate;   ///< 采样率 (Hz)
    uint8_t  bits;          ///< 位深 (bits per sample)
    uint8_t  reserved[5];   ///< 保留字段
};
#pragma pack(pop)

/**
 * @class W25Q64Driver
 * @brief W25Q64 SPI Flash 驱动类
 * @details 通过 SPI 和 CS 引脚操作 W25Q64 Flash 芯片。
 *          支持基本读写擦除以及 DMA 流式读取。
 */
class W25Q64Driver {
public:
    /**
     * @brief 构造函数
     * @param spi SPI 抽象层对象引用
     * @param cs  CS 片选引脚对象引用 (低电平选中)
     */
    W25Q64Driver(hal::Spi& spi, hal::Gpio& cs)
        : spi_(spi), cs_(cs) {}

    /**
     * @brief 初始化 W25Q64
     * @return true 初始化成功 (JEDEC ID 匹配); false 失败
     * @details 释放掉电模式后读取 JEDEC ID 验证芯片身份。
     *          W25Q64JV 的 JEDEC ID = 0xEF4017
     */
    bool init() {
        releasePowerDown();
        HAL_Delay(1);  // 等待芯片唤醒 (tRES1 = 3μs)
        uint32_t id = readJEDEC();
        // W25Q64JV: Manufacturer=0xEF, MemType=0x40, Capacity=0x17
        return (id == 0xEF4017);
    }

    /**
     * @brief 读取 JEDEC ID
     * @return 24-bit JEDEC ID (Manufacturer[23:16] | MemType[15:8] | Capacity[7:0])
     */
    uint32_t readJEDEC() {
        uint8_t cmd = W25Q64Cmd::READ_JEDEC_ID;
        uint8_t id[3] = {0};
        csSelect();
        spi_.transmit(&cmd, 1);
        spi_.receive(id, 3);
        csDeselect();
        return (static_cast<uint32_t>(id[0]) << 16) |
               (static_cast<uint32_t>(id[1]) << 8) |
               id[2];
    }

    /**
     * @brief 读取数据（轮询模式）
     * @param addr  Flash 起始地址 (24-bit)
     * @param data  接收数据缓冲区
     * @param size  读取长度 (bytes)
     * @details 使用标准 Read 指令 (0x03)，最高支持 50MHz SPI 时钟。
     *          由于我们 SPI 配置为 32MHz，可安全使用标准读取。
     */
    void readData(uint32_t addr, uint8_t* data, uint16_t size) {
        uint8_t cmd[4];
        cmd[0] = W25Q64Cmd::READ_DATA;
        cmd[1] = static_cast<uint8_t>((addr >> 16) & 0xFF);  // 地址高字节
        cmd[2] = static_cast<uint8_t>((addr >> 8) & 0xFF);   // 地址中字节
        cmd[3] = static_cast<uint8_t>(addr & 0xFF);           // 地址低字节
        csSelect();
        spi_.transmit(cmd, 4);
        spi_.receive(data, size);
        csDeselect();
    }

    /**
     * @brief DMA 模式读取数据（非阻塞）
     * @param addr  Flash 起始地址 (24-bit)
     * @param data  接收数据缓冲区
     * @param size  读取长度 (bytes)
     * @details 发送命令后使用 DMA 接收数据。
     *          注意：调用后需等待 DMA 完成回调才能释放 CS！
     *          CS 释放需在 DMA 完成回调中调用 csDeselect()。
     */
    void readDataDMA(uint32_t addr, uint8_t* data, uint16_t size) {
        uint8_t cmd[4];
        cmd[0] = W25Q64Cmd::READ_DATA;
        cmd[1] = static_cast<uint8_t>((addr >> 16) & 0xFF);
        cmd[2] = static_cast<uint8_t>((addr >> 8) & 0xFF);
        cmd[3] = static_cast<uint8_t>(addr & 0xFF);
        csSelect();
        spi_.transmit(cmd, 4);       // 命令用轮询发送 (仅4字节)
        spi_.receiveDMA(data, size);  // 数据用 DMA 接收
        // 注意：CS 释放需在 DMA 完成回调中执行
    }

    /** @brief 在 DMA 完成后释放 CS（供回调调用） */
    void csDeselect() {
        cs_.setHigh();
    }

    /**
     * @brief 页编程（写入数据）
     * @param addr Flash 起始地址 (必须页对齐或在页内)
     * @param data 写入数据缓冲区
     * @param size 写入长度 (bytes, 不得超过 256 且不得跨页)
     * @details W25Q64 每次最多写入 256 字节 (一页)，且不能跨页边界。
     *          页边界 = addr & 0xFFFFFF00。
     *          写入前必须先擦除对应扇区！
     */
    void pageProgram(uint32_t addr, const uint8_t* data, uint16_t size) {
        writeEnable();
        uint8_t cmd[4];
        cmd[0] = W25Q64Cmd::PAGE_PROGRAM;
        cmd[1] = static_cast<uint8_t>((addr >> 16) & 0xFF);
        cmd[2] = static_cast<uint8_t>((addr >> 8) & 0xFF);
        cmd[3] = static_cast<uint8_t>(addr & 0xFF);
        csSelect();
        spi_.transmit(cmd, 4);
        spi_.transmit(data, size);
        csDeselect();
        waitBusy();  // 页编程典型时间 0.7ms, 最大 3ms
    }

    /**
     * @brief 扇区擦除 (4KB)
     * @param addr 对应扇区内的任意地址
     * @details 擦除整个 4KB 扇区。擦除后该扇区所有位变为 1 (0xFF)。
     *          典型擦除时间 45ms, 最大 400ms。
     */
    void sectorErase(uint32_t addr) {
        writeEnable();
        uint8_t cmd[4];
        cmd[0] = W25Q64Cmd::SECTOR_ERASE;
        cmd[1] = static_cast<uint8_t>((addr >> 16) & 0xFF);
        cmd[2] = static_cast<uint8_t>((addr >> 8) & 0xFF);
        cmd[3] = static_cast<uint8_t>(addr & 0xFF);
        csSelect();
        spi_.transmit(cmd, 4);
        csDeselect();
        waitBusy();
    }

    /**
     * @brief 块擦除 (64KB)
     * @param addr 对应块内的任意地址
     */
    void blockErase64K(uint32_t addr) {
        writeEnable();
        uint8_t cmd[4];
        cmd[0] = W25Q64Cmd::BLOCK_ERASE_64K;
        cmd[1] = static_cast<uint8_t>((addr >> 16) & 0xFF);
        cmd[2] = static_cast<uint8_t>((addr >> 8) & 0xFF);
        cmd[3] = static_cast<uint8_t>(addr & 0xFF);
        csSelect();
        spi_.transmit(cmd, 4);
        csDeselect();
        waitBusy();
    }

    /**
     * @brief 全片擦除
     * @details 擦除整个 8MB Flash。典型时间 20s, 最大 100s。
     *          慎用！
     */
    void chipErase() {
        writeEnable();
        uint8_t cmd = W25Q64Cmd::CHIP_ERASE;
        csSelect();
        spi_.transmit(&cmd, 1);
        csDeselect();
        waitBusy();
    }

    /**
     * @brief 写入任意长度数据（自动分页处理）
     * @param addr Flash 起始地址
     * @param data 写入数据缓冲区
     * @param size 写入长度 (bytes)
     * @details 自动处理跨页边界的情况，按页拆分写入。
     *          注意：写入前对应区域必须已被擦除！
     */
    void writeData(uint32_t addr, const uint8_t* data, uint32_t size) {
        while (size > 0) {
            // 计算当前页内剩余空间
            // 页边界 = (addr / 256 + 1) * 256
            uint32_t pageRemain = W25Q64Param::PAGE_SIZE - (addr % W25Q64Param::PAGE_SIZE);
            uint16_t writeLen = (size < pageRemain)
                                ? static_cast<uint16_t>(size)
                                : static_cast<uint16_t>(pageRemain);
            pageProgram(addr, data, writeLen);
            addr += writeLen;
            data += writeLen;
            size -= writeLen;
        }
    }

    // ========== 音频索引表操作 ==========

    /**
     * @brief 读取音频索引表头部
     * @param header 输出头部结构体
     * @return true 魔数正确; false 索引表无效
     */
    bool readAudioIndexHeader(AudioIndexHeader& header) {
        readData(AudioIndex::INDEX_BASE_ADDR,
                 reinterpret_cast<uint8_t*>(&header),
                 sizeof(AudioIndexHeader));
        return (header.magic == AudioIndex::MAGIC);
    }

    /**
     * @brief 读取指定索引的音频条目
     * @param index 音频索引 (0 ~ MAX_TRACKS-1)
     * @param entry 输出条目结构体
     * @return true 读取成功且条目有效; false 索引越界或条目无效
     */
    bool readAudioEntry(uint8_t index, AudioIndexEntry& entry) {
        if (index >= AudioIndex::MAX_TRACKS) return false;
        uint32_t addr = AudioIndex::INDEX_BASE_ADDR + AudioIndex::HEADER_SIZE
                        + index * AudioIndex::ENTRY_SIZE;
        readData(addr, reinterpret_cast<uint8_t*>(&entry), sizeof(AudioIndexEntry));
        // 检查条目有效性：起始地址和长度不为 0 或全 0xFF
        return (entry.start_addr != 0 && entry.start_addr != 0xFFFFFFFF &&
                entry.length != 0 && entry.length != 0xFFFFFFFF);
    }

    /**
     * @brief 写入音频索引表（头部 + 全部条目所在的扇区0）
     * @param header 索引表头部
     * @param entries 索引条目数组
     * @param count 条目数量
     * @details 会先擦除 Sector 0，再依次写入头部和各条目
     */
    void writeAudioIndex(const AudioIndexHeader& header,
                         const AudioIndexEntry* entries, uint8_t count) {
        sectorErase(AudioIndex::INDEX_BASE_ADDR);
        // 写入头部
        pageProgram(AudioIndex::INDEX_BASE_ADDR,
                    reinterpret_cast<const uint8_t*>(&header),
                    sizeof(AudioIndexHeader));
        // 写入条目（头部16字节 + 条目紧随其后，共在一个页内）
        if (count > 0 && count <= AudioIndex::MAX_TRACKS) {
            uint32_t entryAddr = AudioIndex::INDEX_BASE_ADDR + AudioIndex::HEADER_SIZE;
            // 所有条目总大小 = count * 16，加上头部16字节 = 最大 32*16+16 = 528 字节
            // 需要拆分为多页写入（256字节一页）
            writeData(entryAddr,
                      reinterpret_cast<const uint8_t*>(entries),
                      static_cast<uint32_t>(count) * AudioIndex::ENTRY_SIZE);
        }
    }

private:
    hal::Spi& spi_;   ///< SPI 抽象层引用
    hal::Gpio& cs_;   ///< CS 片选引脚引用

    /** @brief 拉低 CS 选中芯片 */
    void csSelect() {
        cs_.setLow();
    }

    /** @brief 发送写使能指令 */
    void writeEnable() {
        uint8_t cmd = W25Q64Cmd::WRITE_ENABLE;
        csSelect();
        spi_.transmit(&cmd, 1);
        csDeselect();
    }

    /**
     * @brief 从掉电模式唤醒
     */
    void releasePowerDown() {
        uint8_t cmd = W25Q64Cmd::RELEASE_POWER_DOWN;
        csSelect();
        spi_.transmit(&cmd, 1);
        csDeselect();
    }

    /**
     * @brief 等待 Flash 操作完成（BUSY 位清零）
     * @details 轮询状态寄存器1的 BUSY 位，直到操作完成。
     *          不同操作的等待时间差异很大：
     *          - 页编程：最大 3ms
     *          - 扇区擦除：最大 400ms
     *          - 全片擦除：最大 100s
     */
    void waitBusy() {
        uint8_t cmd = W25Q64Cmd::READ_STATUS_REG1;
        uint8_t status = 0;
        do {
            csSelect();
            spi_.transmit(&cmd, 1);
            spi_.receive(&status, 1);
            csDeselect();
        } while (status & W25Q64Param::STATUS_BUSY_BIT);
    }
};

} // namespace hdl

#endif // W25Q64_DRIVER_HPP
