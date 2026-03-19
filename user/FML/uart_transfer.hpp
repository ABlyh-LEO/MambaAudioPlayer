/**
 * @file    uart_transfer.hpp
 * @brief   UART 音频传输协议模块
 * @details 实现基于 UART 的音频数据传输协议，支持可靠传输。
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
 *      0x01 - START_TRANSFER   开始传输 (上位机→下位机)
 *             DATA 内容: track_index[1B], total_size[4B], sample_rate[2B], bits[1B]
 *      0x02 - DATA_PACKET      数据包 (上位机→下位机)
 *             DATA 内容: 128 字节原始音频数据
 *      0x03 - END_TRANSFER     结束传输 (上位机→下位机)
 *             DATA 内容: 无
 *      0x10 - ACK              确认 (下位机→上位机)
 *             DATA 内容: ack_seq[2B] (已确认的最大连续包序号)
 *      0x11 - NAK              否认/重传请求 (下位机→上位机)
 *             DATA 内容: expected_seq[2B] (期望的包序号)
 *      0x20 - QUERY_LIST       查询音频列表 (上位机→下位机)
 *             DATA 内容: 无
 *      0x21 - LIST_RESPONSE    音频列表响应 (下位机→上位机)
 *             DATA 内容: count[1B], entry[]{index[1B], length[4B], sample_rate[2B]}
 *  - SEQ:     包序号 (Little-Endian), 从 0 开始递增
 *  - LEN:     DATA 字段有效数据长度 (0~128), 无效部分填 0
 *  - CRC16:   CRC-CCITT-FALSE 校验 (Little-Endian)
 *             计算范围: CMD + SEQ + LEN + DATA (共 132 bytes)
 *             多项式: 0x1021, 初始值: 0xFFFF, 异或输出: 0x0000
 *
 *  传输流程：
 *  1. 上位机发送 START_TRANSFER, 包含目标轨道号、总大小等信息
 *  2. 下位机收到后擦除对应 Flash 区域, 回复 ACK
 *  3. 上位机连续发送 DATA_PACKET (SEQ 从 1 递增)
 *  4. 下位机每收到 ACK_INTERVAL(8) 个包, 发送一次 ACK
 *     ACK 中 ack_seq = 已成功接收的最大连续包编号
 *  5. 上位机如果超时未收到 ACK, 从 ack_seq+1 开始重传
 *  6. 传输完成后上位机发送 END_TRANSFER
 *  7. 下位机更新 Flash 索引表, 回复 ACK
 *
 *  错误处理：
 *  - CRC 校验失败: 下位机丢弃该包, 不回复 (上位机超时重传)
 *  - 包序号不连续: 下位机发送 NAK 并指定期望序号
 *  - Flash 写入失败: 下位机发送 NAK
 *
 *  ============================================================================
 */
#ifndef UART_TRANSFER_HPP
#define UART_TRANSFER_HPP

#include "HAL/hal_uart.hpp"
#include "HDL/w25q64_driver.hpp"
#include <cstring>

namespace fml {

/// 协议常量
namespace Protocol {
    constexpr uint8_t  HEADER_0         = 0xAA;
    constexpr uint8_t  HEADER_1         = 0x55;
    constexpr uint8_t  PACKET_DATA_SIZE = 128;    ///< DATA 字段长度
    constexpr uint16_t PACKET_TOTAL_SIZE = 136;   ///< 整包长度
    constexpr uint8_t  ACK_INTERVAL     = 8;      ///< 每 N 个包确认一次

    // 命令定义
    constexpr uint8_t CMD_START_TRANSFER = 0x01;
    constexpr uint8_t CMD_DATA_PACKET    = 0x02;
    constexpr uint8_t CMD_END_TRANSFER   = 0x03;
    constexpr uint8_t CMD_ACK            = 0x10;
    constexpr uint8_t CMD_NAK            = 0x11;
    constexpr uint8_t CMD_QUERY_LIST     = 0x20;
    constexpr uint8_t CMD_LIST_RESPONSE  = 0x21;
}

/// 数据包结构
#pragma pack(push, 1)
struct Packet {
    uint8_t  header[2];                     ///< 固定 0xAA, 0x55
    uint8_t  cmd;                           ///< 命令
    uint16_t seq;                           ///< 包序号
    uint8_t  len;                           ///< DATA 有效长度
    uint8_t  data[Protocol::PACKET_DATA_SIZE]; ///< 数据
    uint16_t crc16;                         ///< CRC16 校验
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
 * @brief UART 音频传输管理类
 * @details 实现上述协议的下位机端。
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
          sampleRate_(16000), bits_(8),
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

        // 重新启动接收
        startReceive();
    }

    /** @brief 获取传输状态 */
    TransferState getState() const { return state_; }

    /** @brief 是否正在传输 */
    bool isTransferring() const { return state_ == TransferState::RECEIVING; }

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

    uint8_t  rxBuf_[RX_BUF_SIZE]; ///< 接收缓冲区
    volatile uint16_t rxLen_;      ///< 已接收数据长度

    /** @brief 启动 UART DMA 接收 */
    void startReceive() {
        uart_.receiveToIdleDMA(rxBuf_, RX_BUF_SIZE);
    }

    /**
     * @brief 处理已验证的数据包
     * @param pkt 解析后的数据包
     */
    void handlePacket(const Packet& pkt) {
        switch (pkt.cmd) {
            case Protocol::CMD_START_TRANSFER:
                handleStartTransfer(pkt);
                break;
            case Protocol::CMD_DATA_PACKET:
                handleDataPacket(pkt);
                break;
            case Protocol::CMD_END_TRANSFER:
                handleEndTransfer(pkt);
                break;
            case Protocol::CMD_QUERY_LIST:
                handleQueryList();
                break;
            default:
                break;
        }
    }

    /** @brief 处理开始传输命令 */
    void handleStartTransfer(const Packet& pkt) {
        if (pkt.len < 8) return;

        trackIndex_ = pkt.data[0];
        memcpy(&totalSize_, &pkt.data[1], 4);
        memcpy(&sampleRate_, &pkt.data[5], 2);
        bits_ = pkt.data[7];

        // 计算写入地址：从音频数据区基地址开始
        // 简单策略：每个轨道分配固定偏移，或者查找空闲位置
        // 这里使用简单的追加策略：从索引表读取已有数据末尾位置
        writeAddr_ = calculateWriteAddress();
        receivedSize_ = 0;
        expectedSeq_ = 1;
        state_ = TransferState::RECEIVING;

        // 擦除所需扇区
        uint32_t sectorsNeeded = (totalSize_ + hdl::W25Q64Param::SECTOR_SIZE - 1)
                                 / hdl::W25Q64Param::SECTOR_SIZE;
        for (uint32_t i = 0; i < sectorsNeeded; ++i) {
            flash_.sectorErase(writeAddr_ + i * hdl::W25Q64Param::SECTOR_SIZE);
        }

        // 发送 ACK
        sendAck(0);
    }

    /** @brief 处理数据包 */
    void handleDataPacket(const Packet& pkt) {
        if (state_ != TransferState::RECEIVING) return;

        if (pkt.seq != expectedSeq_) {
            sendNak(expectedSeq_);
            return;
        }

        // 写入 Flash
        uint16_t dataLen = pkt.len;
        if (dataLen > Protocol::PACKET_DATA_SIZE) dataLen = Protocol::PACKET_DATA_SIZE;

        flash_.writeData(writeAddr_ + receivedSize_, pkt.data, dataLen);
        receivedSize_ += dataLen;
        ++expectedSeq_;

        // 累积确认：每 ACK_INTERVAL 个包发送一次 ACK
        if (expectedSeq_ % Protocol::ACK_INTERVAL == 0) {
            sendAck(pkt.seq);
        }
    }

    /** @brief 处理结束传输命令 */
    void handleEndTransfer(const Packet& pkt) {
        (void)pkt;
        if (state_ != TransferState::RECEIVING) return;

        // 更新音频索引表
        updateAudioIndex();

        state_ = TransferState::COMPLETE;

        // 发送 ACK
        sendAck(expectedSeq_ - 1);

        // 回到空闲状态
        state_ = TransferState::IDLE;
    }

    /** @brief 处理查询音频列表 */
    void handleQueryList() {
        Packet resp;
        memset(&resp, 0, sizeof(resp));
        resp.header[0] = Protocol::HEADER_0;
        resp.header[1] = Protocol::HEADER_1;
        resp.cmd = Protocol::CMD_LIST_RESPONSE;
        resp.seq = 0;

        // 读取索引表
        hdl::AudioIndexHeader header;
        bool valid = flash_.readAudioIndexHeader(header);

        if (valid && header.count > 0) {
            resp.data[0] = header.count;
            uint8_t pos = 1;
            for (uint8_t i = 0; i < header.count && pos + 7 <= Protocol::PACKET_DATA_SIZE; ++i) {
                hdl::AudioIndexEntry entry;
                if (flash_.readAudioEntry(i, entry)) {
                    resp.data[pos++] = i;
                    memcpy(&resp.data[pos], &entry.length, 4);
                    pos += 4;
                    memcpy(&resp.data[pos], &entry.sample_rate, 2);
                    pos += 2;
                }
            }
            resp.len = pos;
        } else {
            resp.data[0] = 0;
            resp.len = 1;
        }

        // 计算 CRC
        resp.crc16 = crc16Ccitt(
            reinterpret_cast<uint8_t*>(&resp) + 2,
            Protocol::PACKET_TOTAL_SIZE - 4
        );

        uart_.transmit(reinterpret_cast<uint8_t*>(&resp), Protocol::PACKET_TOTAL_SIZE);
    }

    /** @brief 发送 ACK 包 */
    void sendAck(uint16_t ackSeq) {
        Packet ack;
        memset(&ack, 0, sizeof(ack));
        ack.header[0] = Protocol::HEADER_0;
        ack.header[1] = Protocol::HEADER_1;
        ack.cmd = Protocol::CMD_ACK;
        ack.seq = 0;
        ack.len = 2;
        memcpy(ack.data, &ackSeq, 2);
        ack.crc16 = crc16Ccitt(
            reinterpret_cast<uint8_t*>(&ack) + 2,
            Protocol::PACKET_TOTAL_SIZE - 4
        );
        uart_.transmit(reinterpret_cast<uint8_t*>(&ack), Protocol::PACKET_TOTAL_SIZE);
    }

    /** @brief 发送 NAK 包 */
    void sendNak(uint16_t expectedSeq) {
        Packet nak;
        memset(&nak, 0, sizeof(nak));
        nak.header[0] = Protocol::HEADER_0;
        nak.header[1] = Protocol::HEADER_1;
        nak.cmd = Protocol::CMD_NAK;
        nak.seq = 0;
        nak.len = 2;
        memcpy(nak.data, &expectedSeq, 2);
        nak.crc16 = crc16Ccitt(
            reinterpret_cast<uint8_t*>(&nak) + 2,
            Protocol::PACKET_TOTAL_SIZE - 4
        );
        uart_.transmit(reinterpret_cast<uint8_t*>(&nak), Protocol::PACKET_TOTAL_SIZE);
    }

    /**
     * @brief 计算新音频的写入地址
     * @return Flash 写入起始地址
     * @details 查找当前索引表中所有音频数据的末尾位置，
     *          新数据从末尾开始写入。
     */
    uint32_t calculateWriteAddress() {
        hdl::AudioIndexHeader header;
        if (!flash_.readAudioIndexHeader(header) || header.count == 0) {
            return hdl::AudioIndex::AUDIO_DATA_BASE;
        }

        uint32_t maxEnd = hdl::AudioIndex::AUDIO_DATA_BASE;
        for (uint8_t i = 0; i < header.count; ++i) {
            hdl::AudioIndexEntry entry;
            if (flash_.readAudioEntry(i, entry)) {
                uint32_t end = entry.start_addr + entry.length;
                if (end > maxEnd) maxEnd = end;
            }
        }

        // 对齐到扇区边界 (4KB)
        maxEnd = (maxEnd + hdl::W25Q64Param::SECTOR_SIZE - 1)
                 & ~(hdl::W25Q64Param::SECTOR_SIZE - 1);
        return maxEnd;
    }

    /** @brief 更新音频索引表 */
    void updateAudioIndex() {
        hdl::AudioIndexHeader header;
        hdl::AudioIndexEntry entries[hdl::AudioIndex::MAX_TRACKS];
        memset(&header, 0, sizeof(header));
        memset(entries, 0xFF, sizeof(entries));

        // 读取现有索引
        bool valid = flash_.readAudioIndexHeader(header);
        if (valid) {
            for (uint8_t i = 0; i < header.count && i < hdl::AudioIndex::MAX_TRACKS; ++i) {
                flash_.readAudioEntry(i, entries[i]);
            }
        } else {
            header.magic = hdl::AudioIndex::MAGIC;
            header.count = 0;
        }

        // 添加或更新当前轨道
        if (trackIndex_ < hdl::AudioIndex::MAX_TRACKS) {
            entries[trackIndex_].start_addr = writeAddr_;
            entries[trackIndex_].length = receivedSize_;
            entries[trackIndex_].sample_rate = sampleRate_;
            entries[trackIndex_].bits = bits_;
            memset(entries[trackIndex_].reserved, 0, sizeof(entries[trackIndex_].reserved));

            if (trackIndex_ >= header.count) {
                header.count = trackIndex_ + 1;
            }
        }

        // 写回索引表
        flash_.writeAudioIndex(header, entries, header.count);
    }

    /**
     * @brief CRC-CCITT-FALSE 计算
     * @param data 数据缓冲区
     * @param len  数据长度
     * @return CRC16 值
     * @details 多项式: 0x1021
     *          初始值: 0xFFFF
     *          输入/输出反转: 否
     *          异或输出: 0x0000
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
