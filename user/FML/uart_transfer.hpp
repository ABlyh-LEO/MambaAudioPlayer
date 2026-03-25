/**
 * @file    uart_transfer.hpp
 * @brief   UART 音频传输与管理协议模块
 * @details 实现基于 UART 的音频数据传输和管理协议，支持可靠传输。
 *
 *  ============================================================================
 *  上位机通信协议定义 (Protocol Specification for Host PC)
 *  ============================================================================
 *
 *  物理层参数：
 *  - 波特率：1000000 (1Mbps)
 *  - 数据位：8
 *  - 停止位：1
 *  - 校验：无
 *
 *  数据包格式 (固定 136 字节)：
 *  ┌──────────┬──────┬──────┬──────┬───────────┬──────────┐
 *  │ HEADER   │ CMD  │ SEQ  │ LEN  │   DATA    │  CRC16   │
 *  │ 2 bytes  │ 1B   │ 2B   │ 1B   │ 128 bytes │  2 bytes │
 *  │ 0xAA55   │      │      │      │(不足补0)  │          │
 *  └──────────┴──────┴──────┴──────┴───────────┴──────────┘
 *
 *  字段说明：
 *  - HEADER:  固定 0xAA, 0x55
 *  - CMD:     命令字节
 *
 *    === 音频传输命令 ===
 *      0x01 - START_TRANSFER   开始传输 (上位机→下位机)
 *             DATA: track_index[1B], total_size[4B], sample_rate[2B], bits[1B]
 *      0x02 - DATA_PACKET      数据包 (上位机→下位机)
 *             DATA: 128 字节原始音频数据
 *      0x03 - END_TRANSFER     结束传输 (上位机→下位机)
 *             DATA: 无
 *
 *    === 通用应答 ===
 *      0x10 - ACK              确认 (下位机→上位机)
 *             DATA: ack_seq[2B] (已确认的最大连续包序号)
 *      0x11 - NAK              否认/错误 (下位机→上位机)
 *             DATA: error_code[1B], detail[2B]
 *             error_code: 0=序号不连续, 1=轨道不存在, 2=Flash空间不足,
 *                         3=参数错误, 4=操作失败
 *
 *    === 音频管理命令 ===
 *      0x20 - QUERY_LIST       查询音频列表 (上位机→下位机)
 *             DATA: 无
 *      0x21 - LIST_RESPONSE    音频列表响应 (下位机→上位机)
 *             DATA: count[1B], entry[]{index[1B], length[4B], sample_rate[2B]}
 *      0x30 - DELETE_TRACK     删除指定音频 (上位机→下位机)
 *             DATA: track_index[1B]
 *             响应: ACK(成功) / NAK(error_code=1, 轨道不存在)
 *             说明: 懒删除，仅清除索引条目，数据留在 Flash 中。
 *                   使用 DEFRAG 命令可回收已删除音频占用的空间。
 *      0x31 - FORMAT_ALL       格式化全部音频 (上位机→下位机)
 *             DATA: 无
 *             响应: ACK(完成)
 *             说明: 擦除索引表(Sector 0)，使所有音频条目失效。
 *                   音频数据区不擦除(节省时间和Flash寿命)。
 *      0x32 - QUERY_STATUS     查询系统状态 (上位机→下位机)
 *             DATA: 无
 *      0x33 - STATUS_RESPONSE  系统状态响应 (下位机→上位机)
 *             DATA: battery_voltage_mv[4B], battery_percent[1B],
 *                   i2c_available[1B], system_state[1B],
 *                   flash_used_bytes[4B], flash_total_bytes[4B],
 *                   track_count[1B]
 *             system_state: 0=Normal, 1=LowBattery, 2=Muted, 3=Transfer
 *      0x34 - DEFRAG           整理Flash碎片 (上位机→下位机)
 *             DATA: 无
 *             响应: ACK(完成)
 *             说明: 将所有有效音频数据紧凑排列到数据区起始位置，
 *                   消除懒删除留下的空洞，并重建索引表(从0开始连续编号)。
 *                   注意: 此操作耗时较长(取决于数据量)，执行期间勿断电！
 *
 *  - SEQ:     包序号 (Little-Endian), 从 0 开始递增
 *  - LEN:     DATA 字段有效数据长度 (0~128), 无效部分填 0
 *  - CRC16:   CRC-CCITT-FALSE 校验 (Little-Endian)
 *             计算范围: CMD + SEQ + LEN + DATA (共 132 bytes)
 *             多项式: 0x1021, 初始值: 0xFFFF, 异或输出: 0x0000
 *
 *  传输流程：
 *  1. 上位机发送 START_TRANSFER, 包含目标轨道号、总大小等信息
 *  2. 下位机在 Flash 中查找足够大的空闲空间(优先复用空洞)
 *     若空间不足则回复 NAK(error_code=2)
 *  3. 下位机擦除目标扇区, 回复 ACK
 *  4. 上位机连续发送 DATA_PACKET (SEQ 从 1 递增)
 *  5. 下位机每收到 ACK_INTERVAL(8) 个包, 发送一次 ACK
 *  6. 传输完成后上位机发送 END_TRANSFER
 *  7. 下位机更新 Flash 索引表, 回复 ACK
 *
 *  删除策略 (懒删除)：
 *  - DELETE_TRACK 仅将索引条目标记为无效(擦写Sector 0)
 *  - 已删除音频的数据仍保留在 Flash 中(不擦除)
 *  - 上传新音频时, 自动扫描地址空间中的空洞并复用
 *  - DEFRAG 命令可将所有有效数据紧凑排列并回收空间
 *
 *  ============================================================================
 */
#ifndef UART_TRANSFER_HPP
#define UART_TRANSFER_HPP

#include "HAL/hal_uart.hpp"
#include "HDL/w25q64_driver.hpp"
#include <cstring>

namespace fml {

/// 系统状态快照 (由 AppMain 更新, UartTransfer 读取)
struct SystemStatusInfo {
    uint32_t battery_voltage_mv = 0;
    uint8_t battery_percent = 0;
    uint8_t i2c_available = 0;
    uint8_t system_state = 0;   ///< 0=Normal,1=LowBatt,2=Muted,3=Transfer
};

/// NAK 错误码
namespace NakError {
    constexpr uint8_t SEQ_MISMATCH    = 0;  ///< 包序号不连续
    constexpr uint8_t TRACK_NOT_FOUND = 1;  ///< 轨道不存在
    constexpr uint8_t NO_SPACE        = 2;  ///< Flash 空间不足
    constexpr uint8_t BAD_PARAM       = 3;  ///< 参数错误
    constexpr uint8_t OP_FAILED       = 4;  ///< 操作失败
}

/// 协议常量
namespace Protocol {
    constexpr uint8_t  HEADER_0         = 0xAA;
    constexpr uint8_t  HEADER_1         = 0x55;
    constexpr uint8_t  PACKET_DATA_SIZE = 128;    ///< DATA 字段长度
    constexpr uint16_t PACKET_TOTAL_SIZE = 136;   ///< 整包长度
    constexpr uint8_t  ACK_INTERVAL     = 1;      ///< 包级同步：每发送一个包确认一次，避免MCU擦除扇区时丢失数据

    // === 音频传输命令 ===
    constexpr uint8_t CMD_START_TRANSFER = 0x01;
    constexpr uint8_t CMD_DATA_PACKET    = 0x02;
    constexpr uint8_t CMD_END_TRANSFER   = 0x03;

    // === 通用应答 ===
    constexpr uint8_t CMD_ACK            = 0x10;
    constexpr uint8_t CMD_NAK            = 0x11;

    // === 音频管理命令 ===
    constexpr uint8_t CMD_QUERY_LIST     = 0x20;
    constexpr uint8_t CMD_LIST_RESPONSE  = 0x21;
    constexpr uint8_t CMD_DELETE_TRACK   = 0x30;
    constexpr uint8_t CMD_FORMAT_ALL     = 0x31;
    constexpr uint8_t CMD_QUERY_STATUS   = 0x32;
    constexpr uint8_t CMD_STATUS_RESPONSE= 0x33;
    constexpr uint8_t CMD_DEFRAG         = 0x34;
}

/// 数据包结构
#pragma pack(push, 1)
struct Packet {
    uint8_t  header[2];                        ///< 固定 0xAA, 0x55
    uint8_t  cmd;                              ///< 命令
    uint16_t seq;                              ///< 包序号
    uint8_t  len;                              ///< DATA 有效长度
    uint8_t  data[Protocol::PACKET_DATA_SIZE]; ///< 数据
    uint16_t crc16;                            ///< CRC16 校验
};
#pragma pack(pop)

/// 传输状态枚举
enum class TransferState {
    IDLE,           ///< 空闲
    RECEIVING,      ///< 正在接收数据
    COMPLETE        ///< 传输完成
};

/**
 * @class UartTransfer
 * @brief UART 音频传输与管理类
 * @details 实现上述协议的下位机端。
 *          支持音频上传、删除、格式化、状态查询和碎片整理。
 */
class UartTransfer {
public:
    /// UART 接收缓冲区大小
    static constexpr uint16_t RX_BUF_SIZE = 256;

    /**
     * @brief 构造函数
     * @param uart  UART 抽象层引用
     * @param flash W25Q64 驱动引用
     */
    UartTransfer(hal::Uart& uart, hdl::W25Q64Driver& flash)
        : uart_(uart), flash_(flash),
          state_(TransferState::IDLE),
          expectedSeq_(0), trackIndex_(0),
          writeAddr_(0), totalSize_(0), receivedSize_(0),
          sampleRate_(16000), bits_(8), lastErasedSector_(0xFFFFFFFF),
          rxLen_(0) {
        memset(rxBuf_, 0, sizeof(rxBuf_));
    }

    /**
     * @brief 初始化 UART 接收
     * @details 启动 DMA 空闲中断接收
     */
    void init() {
        startReceive();
    }

    /**
     * @brief UART 接收完成回调 (在 HAL 回调中调用)
     * @param size 本次接收到的数据长度
     */
    void onReceiveComplete(uint16_t size) {
        rxLen_ = size;
    }

    /**
     * @brief 主循环中处理接收到的数据
     * @details 检查是否有新数据到达，解析并处理
     */
    void processReceive() {
        if (rxLen_ == 0) return;

        uint16_t len = rxLen_;
        rxLen_ = 0;

        // 检查是否为完整数据包
        if (len >= Protocol::PACKET_TOTAL_SIZE) {
            Packet* pkt = reinterpret_cast<Packet*>(rxBuf_);

            // 验证包头
            if (pkt->header[0] == Protocol::HEADER_0 &&
                pkt->header[1] == Protocol::HEADER_1) {

                // 验证 CRC16
                uint16_t calcCrc = crc16Ccitt(
                    &rxBuf_[2],  // 从 CMD 开始
                    Protocol::PACKET_TOTAL_SIZE - 4  // 减去 HEADER 和 CRC
                );

                if (calcCrc == pkt->crc16) {
                    handlePacket(*pkt);
                }
                // CRC 失败则丢弃，不回复
            }
        }

        // 处理完成后重新启动接收
        startReceive();
    }

    /** @brief 获取传输状态 */
    TransferState getState() const { return state_; }

    /** @brief 是否正在传输 */
    bool isTransferring() const { return state_ == TransferState::RECEIVING; }

    /**
     * @brief 更新系统状态信息 (由 AppMain 调用)
     * @param voltage  电池电压 (mV)
     * @param percent  电量百分比
     * @param i2cOk    I2C 电池架是否在线
     * @param state    系统状态 (0~3)
     */
    void updateStatus(uint32_t voltage, uint8_t percent, bool i2cOk, uint8_t state) {
        statusInfo_.battery_voltage_mv = voltage;
        statusInfo_.battery_percent = percent;
        statusInfo_.i2c_available = i2cOk ? 1 : 0;
        statusInfo_.system_state = state;
    }

private:
    hal::Uart& uart_;             ///< UART 引用
    hdl::W25Q64Driver& flash_;    ///< Flash 驱动引用

    TransferState state_;          ///< 当前传输状态
    uint16_t expectedSeq_;         ///< 期望的下一个包序号
    uint8_t  trackIndex_;          ///< 当前传输的轨道索引
    uint32_t writeAddr_;           ///< 当前 Flash 写入地址
    uint32_t totalSize_;           ///< 总传输大小
    uint32_t receivedSize_;        ///< 已接收大小
    uint16_t sampleRate_;          ///< 采样率
    uint8_t  bits_;                ///< 位深
    uint32_t lastErasedSector_;    ///< 最近擦除的扇区地址 (按需擦除用)

    uint8_t  rxBuf_[RX_BUF_SIZE]; ///< 接收缓冲区
    volatile uint16_t rxLen_;      ///< 已接收数据长度
    SystemStatusInfo statusInfo_;  ///< 系统状态快照

    /** @brief 启动一次新的数据接收 (定长 136 字节) */
    void startReceive() {
        HAL_StatusTypeDef status = uart_.receiveDMA(rxBuf_, Protocol::PACKET_TOTAL_SIZE);
        if (status != HAL_OK) {
            uart_.clearErrors();
            uart_.receiveDMA(rxBuf_, Protocol::PACKET_TOTAL_SIZE);
        }
    }

    // ====================================================================
    //  统一应答构造与发送
    // ====================================================================

    /**
     * @brief 发送响应包 (所有应答的统一出口)
     * @param cmd  命令字节
     * @param data 响应数据 (可为 nullptr)
     * @param len  响应数据有效长度 (0~128)
     */
    void sendResponse(uint8_t cmd, const uint8_t* data, uint8_t len) {
        Packet resp;
        memset(&resp, 0, sizeof(resp));
        resp.header[0] = Protocol::HEADER_0;
        resp.header[1] = Protocol::HEADER_1;
        resp.cmd = cmd;
        resp.seq = 0;
        resp.len = len;
        if (data && len > 0) {
            memcpy(resp.data, data, len);
        }
        resp.crc16 = crc16Ccitt(
            reinterpret_cast<uint8_t*>(&resp) + 2,
            Protocol::PACKET_TOTAL_SIZE - 4
        );
        uart_.transmit(reinterpret_cast<uint8_t*>(&resp), Protocol::PACKET_TOTAL_SIZE);
        // 等待 UART 发送完毕 (TC 标志), 避免紧接着 re-arm DMA 时 UART 状态异常
        while (!__HAL_UART_GET_FLAG(uart_.getHandle(), UART_FLAG_TC)) {}
    }

    /** @brief 发送 ACK */
    void sendAck(uint16_t ackSeq) {
        sendResponse(Protocol::CMD_ACK,
                     reinterpret_cast<const uint8_t*>(&ackSeq), 2);
    }

    /**
     * @brief 发送 NAK (含错误码)
     * @param errorCode 错误码 (见 NakError 命名空间)
     * @param detail    附加信息 (如期望的 SEQ)
     */
    void sendNak(uint8_t errorCode, uint16_t detail = 0) {
        uint8_t buf[3];
        buf[0] = errorCode;
        memcpy(&buf[1], &detail, 2);
        sendResponse(Protocol::CMD_NAK, buf, 3);
    }

    // ====================================================================
    //  命令分发
    // ====================================================================

    /** @brief 分发已验证的数据包到对应 handler */
    void handlePacket(const Packet& pkt) {
        switch (pkt.cmd) {
            // 传输命令
            case Protocol::CMD_START_TRANSFER: handleStartTransfer(pkt); break;
            case Protocol::CMD_DATA_PACKET:    handleDataPacket(pkt);    break;
            case Protocol::CMD_END_TRANSFER:   handleEndTransfer(pkt);   break;
            // 管理命令
            case Protocol::CMD_QUERY_LIST:     handleQueryList();        break;
            case Protocol::CMD_DELETE_TRACK:   handleDeleteTrack(pkt);   break;
            case Protocol::CMD_FORMAT_ALL:     handleFormatAll();        break;
            case Protocol::CMD_QUERY_STATUS:   handleQueryStatus();      break;
            case Protocol::CMD_DEFRAG:         handleDefrag();           break;
            default: break;
        }
    }

    // ====================================================================
    //  传输命令 handlers
    // ====================================================================

    /** @brief 处理开始传输命令 */
    void handleStartTransfer(const Packet& pkt) {
        if (pkt.len < 8) { sendNak(NakError::BAD_PARAM); return; }

        trackIndex_ = pkt.data[0];
        memcpy(&totalSize_, &pkt.data[1], 4);
        memcpy(&sampleRate_, &pkt.data[5], 2);
        bits_ = pkt.data[7];

        if (trackIndex_ >= hdl::AudioIndex::MAX_TRACKS) {
            sendNak(NakError::BAD_PARAM);
            return;
        }

        // 限制最大传输大小 (不超过 Flash 数据区容量)
        uint32_t maxDataSize = hdl::W25Q64Param::TOTAL_SIZE
                               - hdl::AudioIndex::AUDIO_DATA_BASE;
        if (totalSize_ == 0 || totalSize_ > maxDataSize) {
            sendNak(NakError::BAD_PARAM);
            return;
        }

        // 查找可用空间 (排除当前 trackIndex 以便覆盖上传时复用其空间)
        writeAddr_ = findWriteAddress(totalSize_, trackIndex_);
        if (writeAddr_ == 0xFFFFFFFF) {
            sendNak(NakError::NO_SPACE);
            return;
        }

        receivedSize_ = 0;
        expectedSeq_ = 1;
        lastErasedSector_ = 0xFFFFFFFF;
        state_ = TransferState::RECEIVING;

        // 先回复 ACK (不预擦除扇区, 改为在 handleDataPacket 中按需擦除)
        sendAck(0);
    }

    /** @brief 处理数据包 */
    void handleDataPacket(const Packet& pkt) {
        if (state_ != TransferState::RECEIVING) return;

        if (pkt.seq != expectedSeq_) {
            sendNak(NakError::SEQ_MISMATCH, expectedSeq_);
            return;
        }

        uint16_t dataLen = pkt.len;
        if (dataLen > Protocol::PACKET_DATA_SIZE) dataLen = Protocol::PACKET_DATA_SIZE;

        // 按需擦除：检查即将写入的地址范围是否覆盖了新的扇区
        uint32_t writeStart = writeAddr_ + receivedSize_;
        uint32_t writeEnd   = writeStart + dataLen;
        for (uint32_t addr = writeStart; addr < writeEnd;
             addr = (addr & ~(hdl::W25Q64Param::SECTOR_SIZE - 1))
                    + hdl::W25Q64Param::SECTOR_SIZE) {
            uint32_t sector = addr & ~(hdl::W25Q64Param::SECTOR_SIZE - 1);
            if (sector != lastErasedSector_) {
                flash_.sectorErase(sector);
                lastErasedSector_ = sector;
            }
        }

        flash_.writeData(writeStart, pkt.data, dataLen);
        receivedSize_ += dataLen;
        ++expectedSeq_;

        // 累积确认
        if (expectedSeq_ % Protocol::ACK_INTERVAL == 0) {
            sendAck(pkt.seq);
        }
    }

    /** @brief 处理结束传输命令 */
    void handleEndTransfer(const Packet& pkt) {
        (void)pkt;
        if (state_ != TransferState::RECEIVING) return;

        updateAudioIndex();
        sendAck(expectedSeq_ - 1);
        state_ = TransferState::IDLE;
    }

    // ====================================================================
    //  管理命令 handlers
    // ====================================================================

    /** @brief 处理查询音频列表 */
    void handleQueryList() {
        uint8_t buf[Protocol::PACKET_DATA_SIZE];
        memset(buf, 0, sizeof(buf));

        hdl::AudioIndexHeader header;
        bool valid = flash_.readAudioIndexHeader(header);
        uint8_t pos = 0;

        if (valid && header.count > 0) {
            buf[pos++] = 0;  // 占位，后填实际有效数
            uint8_t validCount = 0;
            for (uint8_t i = 0; i < header.count &&
                 pos + 7 <= Protocol::PACKET_DATA_SIZE; ++i) {
                hdl::AudioIndexEntry entry;
                if (flash_.readAudioEntry(i, entry)) {
                    buf[pos++] = i;
                    memcpy(&buf[pos], &entry.length, 4); pos += 4;
                    memcpy(&buf[pos], &entry.sample_rate, 2); pos += 2;
                    ++validCount;
                }
            }
            buf[0] = validCount;
        } else {
            buf[pos++] = 0;
        }

        sendResponse(Protocol::CMD_LIST_RESPONSE, buf, pos);
    }

    /**
     * @brief 处理删除指定音频 (懒删除)
     * @details 仅擦写 Sector 0 上的索引表，将目标条目标记为无效。
     *          音频数据保留在 Flash 中，不擦除。
     */
    void handleDeleteTrack(const Packet& pkt) {
        if (pkt.len < 1) { sendNak(NakError::BAD_PARAM); return; }
        uint8_t targetIdx = pkt.data[0];

        hdl::AudioIndexHeader header;
        if (!flash_.readAudioIndexHeader(header) || targetIdx >= header.count) {
            sendNak(NakError::TRACK_NOT_FOUND);
            return;
        }

        // 验证目标条目存在
        hdl::AudioIndexEntry testEntry;
        if (!flash_.readAudioEntry(targetIdx, testEntry)) {
            sendNak(NakError::TRACK_NOT_FOUND);
            return;
        }

        // 读取全部条目
        hdl::AudioIndexEntry entries[hdl::AudioIndex::MAX_TRACKS];
        memset(entries, 0xFF, sizeof(entries));
        for (uint8_t i = 0; i < header.count; ++i) {
            flash_.readAudioEntry(i, entries[i]);
        }

        // 将目标条目标记为无效 (全 0xFF = 已擦除状态)
        memset(&entries[targetIdx], 0xFF, sizeof(hdl::AudioIndexEntry));

        // 重新计算 count (最高有效索引 + 1)
        uint8_t newCount = 0;
        for (int8_t i = static_cast<int8_t>(header.count) - 1; i >= 0; --i) {
            if (entries[i].start_addr != 0xFFFFFFFF &&
                entries[i].length != 0xFFFFFFFF) {
                newCount = static_cast<uint8_t>(i) + 1;
                break;
            }
        }
        header.count = newCount;

        // 写回索引表
        flash_.writeAudioIndex(header, entries, (newCount > 0) ? newCount : 0);
        sendAck(0);
    }

    /**
     * @brief 处理格式化全部音频
     * @details 仅擦除 Sector 0 (索引表)，音频数据区不擦除。
     */
    void handleFormatAll() {
        hdl::AudioIndexHeader header;
        header.magic = hdl::AudioIndex::MAGIC;
        header.count = 0;
        memset(header.reserved, 0, sizeof(header.reserved));

        // 擦除 Sector 0 并写入空索引表
        flash_.sectorErase(hdl::AudioIndex::INDEX_BASE_ADDR);
        flash_.writeData(hdl::AudioIndex::INDEX_BASE_ADDR,
                         reinterpret_cast<const uint8_t*>(&header),
                         sizeof(hdl::AudioIndexHeader));
        sendAck(0);
    }

    /** @brief 处理查询系统状态 */
    void handleQueryStatus() {
        uint8_t buf[16];
        uint8_t pos = 0;

        memcpy(&buf[pos], &statusInfo_.battery_voltage_mv, 4); pos += 4;
        buf[pos++] = statusInfo_.battery_percent;
        buf[pos++] = statusInfo_.i2c_available;
        buf[pos++] = statusInfo_.system_state;

        // Flash 使用情况
        uint32_t usedBytes = calculateFlashUsed();
        uint32_t totalBytes = hdl::W25Q64Param::TOTAL_SIZE
                              - hdl::AudioIndex::AUDIO_DATA_BASE;
        memcpy(&buf[pos], &usedBytes, 4);  pos += 4;
        memcpy(&buf[pos], &totalBytes, 4); pos += 4;

        // 轨道数
        hdl::AudioIndexHeader header;
        uint8_t trackCount = 0;
        if (flash_.readAudioIndexHeader(header)) {
            for (uint8_t i = 0; i < header.count; ++i) {
                hdl::AudioIndexEntry e;
                if (flash_.readAudioEntry(i, e)) ++trackCount;
            }
        }
        buf[pos++] = trackCount;

        sendResponse(Protocol::CMD_STATUS_RESPONSE, buf, pos);
    }

    /**
     * @brief 处理碎片整理 (DEFRAG)
     * @details 将所有有效音频数据紧凑排列，重建索引表。
     *          算法：
     *          1. 读取全部有效条目并按地址排序
     *          2. 从 AUDIO_DATA_BASE 起逐个搬移数据 (page-by-page)
     *          3. 重建索引表，有效条目从 index 0 开始连续编号
     *
     *          安全性：所有地址均扇区对齐，搬移方向为从高地址到低地址，
     *          处理顺序从左到右，已读过的数据才会被覆盖。
     *
     *          ⚠ 此操作期间不可断电，否则可能导致数据丢失。
     */
    void handleDefrag() {
        hdl::AudioIndexHeader header;
        if (!flash_.readAudioIndexHeader(header) || header.count == 0) {
            sendAck(0);  // 无数据，无需整理
            return;
        }

        // 收集有效条目
        hdl::AudioIndexEntry entries[hdl::AudioIndex::MAX_TRACKS];
        uint8_t validMap[hdl::AudioIndex::MAX_TRACKS]; // 有效条目的原始索引
        uint8_t validCount = 0;

        for (uint8_t i = 0; i < header.count && i < hdl::AudioIndex::MAX_TRACKS; ++i) {
            if (flash_.readAudioEntry(i, entries[i])) {
                validMap[validCount++] = i;
            }
        }

        if (validCount == 0) {
            handleFormatAll();  // 全部已删除，直接格式化
            return;
        }

        // 按起始地址排序 (insertion sort, 最多 32 个元素)
        for (uint8_t i = 1; i < validCount; ++i) {
            uint8_t key = validMap[i];
            int8_t j = static_cast<int8_t>(i) - 1;
            while (j >= 0 &&
                   entries[validMap[j]].start_addr > entries[key].start_addr) {
                validMap[j + 1] = validMap[j];
                --j;
            }
            validMap[j + 1] = key;
        }

        // 逐个搬移数据，紧凑排列
        uint32_t nextAddr = hdl::AudioIndex::AUDIO_DATA_BASE;
        uint8_t pageBuf[256];

        for (uint8_t v = 0; v < validCount; ++v) {
            uint8_t idx = validMap[v];
            uint32_t oldAddr = entries[idx].start_addr;
            uint32_t length  = entries[idx].length;

            // 对齐到扇区边界
            nextAddr = alignToSector(nextAddr);

            if (nextAddr != oldAddr && nextAddr < oldAddr) {
                // 需要搬移：page-by-page 从低到高复制
                uint32_t remaining = length;
                uint32_t offset = 0;
                uint32_t lastErasedSector = 0xFFFFFFFF;

                while (remaining > 0) {
                    uint16_t chunk = (remaining >= 256)
                                     ? 256
                                     : static_cast<uint16_t>(remaining);

                    // 按需擦除目标扇区
                    uint32_t targetSector = (nextAddr + offset)
                                            & ~(hdl::W25Q64Param::SECTOR_SIZE - 1);
                    if (targetSector != lastErasedSector) {
                        flash_.sectorErase(targetSector);
                        lastErasedSector = targetSector;
                    }

                    flash_.readData(oldAddr + offset, pageBuf, chunk);
                    flash_.writeData(nextAddr + offset, pageBuf, chunk);

                    offset += chunk;
                    remaining -= chunk;
                }
                entries[idx].start_addr = nextAddr;
            }

            nextAddr = entries[idx].start_addr + length;
        }

        // 重建索引表：有效条目从 index 0 开始连续编号
        // 复用 pageBuf 做临时暂存 (256B, 可存 16 个 entry)
        hdl::AudioIndexEntry newEntries[hdl::AudioIndex::MAX_TRACKS];
        memset(newEntries, 0xFF, sizeof(newEntries));
        for (uint8_t v = 0; v < validCount; ++v) {
            newEntries[v] = entries[validMap[v]];
        }

        hdl::AudioIndexHeader newHeader;
        newHeader.magic = hdl::AudioIndex::MAGIC;
        newHeader.count = validCount;
        memset(newHeader.reserved, 0, sizeof(newHeader.reserved));
        flash_.writeAudioIndex(newHeader, newEntries, validCount);

        sendAck(0);
    }

    // ====================================================================
    //  Flash 地址分配
    // ====================================================================

    /**
     * @brief 查找可用的写入地址 (空洞优先 + 尾部追加)
     * @param neededSize 需要的空间大小 (bytes)
     * @param excludeIdx 排除的轨道索引 (覆盖上传时忽略旧数据, -1=不排除)
     * @return Flash 写入起始地址 (扇区对齐), 0xFFFFFFFF 表示空间不足
     *
     * @details 地址分配策略 (First-Fit):
     *          1. 收集所有有效条目 (排除 excludeIdx), 按地址排序
     *          2. 扫描相邻条目之间的空洞
     *          3. 第一个足够大的空洞 → 使用
     *          4. 所有空洞都不够大 → 追加到末尾
     *          5. 末尾空间也不够 → 返回 0xFFFFFFFF (空间不足)
     *
     *          当新音频大于某个空洞时，跳过该空洞继续检查下一个；
     *          若所有空洞均不够大，则使用尾部剩余空间。
     */
    uint32_t findWriteAddress(uint32_t neededSize, int8_t excludeIdx = -1) {
        uint32_t neededAligned = alignToSector(neededSize);

        hdl::AudioIndexHeader header;
        if (!flash_.readAudioIndexHeader(header) || header.count == 0) {
            // 索引为空，从数据区起始位置开始
            if (hdl::AudioIndex::AUDIO_DATA_BASE + neededAligned
                <= hdl::W25Q64Param::TOTAL_SIZE) {
                return hdl::AudioIndex::AUDIO_DATA_BASE;
            }
            return 0xFFFFFFFF;
        }

        // 收集有效条目 {addr, endAddr}，排除 excludeIdx
        struct Span { uint32_t addr; uint32_t end; };
        Span spans[hdl::AudioIndex::MAX_TRACKS];
        uint8_t spanCount = 0;

        for (uint8_t i = 0; i < header.count && i < hdl::AudioIndex::MAX_TRACKS; ++i) {
            if (static_cast<int8_t>(i) == excludeIdx) continue;
            hdl::AudioIndexEntry entry;
            if (flash_.readAudioEntry(i, entry)) {
                spans[spanCount].addr = entry.start_addr;
                spans[spanCount].end  = alignToSector(entry.start_addr + entry.length);
                ++spanCount;
            }
        }

        if (spanCount == 0) {
            if (hdl::AudioIndex::AUDIO_DATA_BASE + neededAligned
                <= hdl::W25Q64Param::TOTAL_SIZE) {
                return hdl::AudioIndex::AUDIO_DATA_BASE;
            }
            return 0xFFFFFFFF;
        }

        // 按地址排序 (insertion sort)
        for (uint8_t i = 1; i < spanCount; ++i) {
            Span key = spans[i];
            int8_t j = static_cast<int8_t>(i) - 1;
            while (j >= 0 && spans[j].addr > key.addr) {
                spans[j + 1] = spans[j];
                --j;
            }
            spans[j + 1] = key;
        }

        // 扫描空洞 (First-Fit)
        // 检查 AUDIO_DATA_BASE 到第一个条目之间的空洞
        uint32_t gapStart = hdl::AudioIndex::AUDIO_DATA_BASE;
        for (uint8_t i = 0; i < spanCount; ++i) {
            uint32_t gapEnd = spans[i].addr;
            if (gapEnd > gapStart && gapEnd - gapStart >= neededAligned) {
                return gapStart;  // 找到足够大的空洞
            }
            // 跳过此条目，更新 gapStart 为此条目结束位置
            if (spans[i].end > gapStart) {
                gapStart = spans[i].end;
            }
        }

        // 检查最后一个条目之后的空间
        if (gapStart + neededAligned <= hdl::W25Q64Param::TOTAL_SIZE) {
            return gapStart;
        }

        return 0xFFFFFFFF;  // Flash 空间不足
    }

    // ====================================================================
    //  辅助函数
    // ====================================================================

    /** @brief 更新音频索引表 (传输完成后调用) */
    void updateAudioIndex() {
        hdl::AudioIndexHeader header;
        hdl::AudioIndexEntry entries[hdl::AudioIndex::MAX_TRACKS];
        memset(entries, 0xFF, sizeof(entries));

        bool valid = flash_.readAudioIndexHeader(header);
        if (valid) {
            for (uint8_t i = 0; i < header.count &&
                 i < hdl::AudioIndex::MAX_TRACKS; ++i) {
                flash_.readAudioEntry(i, entries[i]);
            }
        } else {
            header.magic = hdl::AudioIndex::MAGIC;
            header.count = 0;
            memset(header.reserved, 0, sizeof(header.reserved));
        }

        // 添加或更新当前轨道
        if (trackIndex_ < hdl::AudioIndex::MAX_TRACKS) {
            entries[trackIndex_].start_addr = writeAddr_;
            entries[trackIndex_].length = receivedSize_;
            entries[trackIndex_].sample_rate = sampleRate_;
            entries[trackIndex_].bits = bits_;
            memset(entries[trackIndex_].reserved, 0,
                   sizeof(entries[trackIndex_].reserved));

            if (trackIndex_ >= header.count) {
                header.count = trackIndex_ + 1;
            }
        }

        flash_.writeAudioIndex(header, entries, header.count);
    }

    /** @brief 计算 Flash 音频数据已用空间 (字节) */
    uint32_t calculateFlashUsed() {
        hdl::AudioIndexHeader header;
        if (!flash_.readAudioIndexHeader(header) || header.count == 0) return 0;

        uint32_t total = 0;
        for (uint8_t i = 0; i < header.count; ++i) {
            hdl::AudioIndexEntry e;
            if (flash_.readAudioEntry(i, e)) {
                total += alignToSector(e.length);
            }
        }
        return total;
    }

    /** @brief 地址/大小向上对齐到扇区边界 (4KB) */
    static uint32_t alignToSector(uint32_t val) {
        return (val + hdl::W25Q64Param::SECTOR_SIZE - 1)
               & ~(hdl::W25Q64Param::SECTOR_SIZE - 1);
    }

    /**
     * @brief CRC-CCITT-FALSE 计算
     * @param data 数据缓冲区
     * @param len  数据长度
     * @return CRC16 值
     * @details 多项式: 0x1021, 初始值: 0xFFFF, 输入/输出不反转
     */
    static uint16_t crc16Ccitt(const uint8_t* data, uint16_t len) {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < len; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }
};

} // namespace fml

#endif // UART_TRANSFER_HPP
