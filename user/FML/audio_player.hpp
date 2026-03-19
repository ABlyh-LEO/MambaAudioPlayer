/**
 * @file    audio_player.hpp
 * @brief   音频播放功能模块
 * @details 基于 PWM 模拟 DAC 的音频播放器，从 SPI Flash 读取 8-bit PCM 数据。
 *
 *          音频规格：
 *          - 采样率：16kHz (由 TIM1 PWM 频率决定)
 *          - 位深：8-bit unsigned PCM
 *          - 声道：单声道
 *
 *          双缓冲机制：
 *          ┌──────────┐    ┌──────────┐
 *          │ Buffer A │    │ Buffer B │
 *          │ 256 bytes│    │ 256 bytes│
 *          └────┬─────┘    └────┬─────┘
 *               │               │
 *          当 A 正在播放时，从 Flash 预读数据填充 B
 *          当 A 播完切换到 B 时，再从 Flash 预读数据填充 A
 *
 *          播放流程：
 *          1. playTrack() 从索引表获取音频起始地址和长度
 *          2. 预读第一批数据到 Buffer A
 *          3. 启动 TIM1 Update 中断 (16kHz)
 *          4. 每次中断从当前 Buffer 取一个样本设置 PWM CCR
 *          5. 当前 Buffer 用完时切换到另一个 Buffer
 *          6. 同时触发从 Flash 读取下一批数据到空闲 Buffer
 *          7. 全部数据播完则停止或循环
 *
 *          音量保护：
 *          - PWM 最大占空比限制为 80%，防止扬声器(额定1W,最大1.5W)过载
 *          - volume_scale_ 范围 0~80，默认 60 (约75%音量)
 */
#ifndef AUDIO_PLAYER_HPP
#define AUDIO_PLAYER_HPP

#include "HDL/w25q64_driver.hpp"
#include "HDL/amp_control.hpp"
#include "HAL/hal_pwm.hpp"
#include <cstring>
#include <cmath>

namespace fml {

/**
 * @class AudioPlayer
 * @brief 音频播放管理器
 * @details 管理音频的读取、解码和 PWM 输出。
 *          支持播放 Flash 中存储的音频轨道和纯音播放（用于报警）。
 */
class AudioPlayer {
public:
    /// 双缓冲区大小 (每个 256 字节，总计 512 字节 RAM)
    static constexpr uint16_t BUFFER_SIZE = 256;

    /// 音量缩放最大值 (80% 占空比限制，保护扬声器)
    /// 最大 CCR = sample * 125 / 8 * volume / 100
    /// 当 volume=80 时，最大 CCR = 255 * 125 / 8 * 80 / 100 = 3187 (< 4000)
    static constexpr uint8_t MAX_VOLUME = 80;

    /// 默认音量
    static constexpr uint8_t DEFAULT_VOLUME = 60;

    /**
     * @brief 构造函数
     * @param flash  W25Q64 驱动引用
     * @param amp    功放控制引用
     * @param pwm    PWM 输出引用
     */
    AudioPlayer(hdl::W25Q64Driver& flash, hdl::AmpControl& amp, hal::Pwm& pwm)
        : flash_(flash), amp_(amp), pwm_(pwm),
          playing_(false), looping_(false),
          volume_(DEFAULT_VOLUME),
          currentAddr_(0), remainBytes_(0),
          bufIndex_(0), bufPos_(0), bufReady_{false, false},
          toneMode_(false), tonePhase_(0), tonePeriodSamples_(0),
          toneSamplesLeft_(0), needFill_(false) {
        memset(buffer_, 0, sizeof(buffer_));
    }

    /**
     * @brief 播放指定轨道 (从 Flash 索引表查找)
     * @param trackIndex 轨道索引 (0~31)
     * @param loop       是否循环播放
     * @return true 开始播放; false 轨道不存在或无效
     */
    bool playTrack(uint8_t trackIndex, bool loop = false) {
        hdl::AudioIndexEntry entry;
        if (!flash_.readAudioEntry(trackIndex, entry)) {
            return false;
        }

        // 停止当前播放
        stopInternal();

        // 设置播放参数
        trackStartAddr_ = entry.start_addr;
        trackLength_    = entry.length;
        currentAddr_    = entry.start_addr;
        remainBytes_    = entry.length;
        looping_        = loop;
        toneMode_       = false;

        // 预填充两个缓冲区
        fillBuffer(0);
        fillBuffer(1);
        bufIndex_ = 0;
        bufPos_   = 0;

        // 启动播放
        amp_.enable();
        playing_ = true;
        pwm_.startWithIT();

        return true;
    }

    /**
     * @brief 播放纯音 (正弦波)
     * @param frequency 频率 (Hz, 如 440 为标准 A 音)
     * @param durationMs 持续时间 (ms, 0 = 无限)
     * @details 在没有有效音频数据时用纯音作为报警信号。
     *          正弦波查表法生成：
     *          - 采样率 = 16000 Hz
     *          - 每个周期的采样点数 = 16000 / frequency
     *          - 例如 440Hz: 16000/440 ≈ 36.36 个采样点
     */
    void playTone(uint16_t frequency, uint32_t durationMs = 0) {
        stopInternal();

        toneMode_ = true;
        tonePhase_ = 0;
        // 每个周期的采样点数 (16kHz / frequency)
        tonePeriodSamples_ = 16000 / frequency;
        if (tonePeriodSamples_ == 0) tonePeriodSamples_ = 1;

        // 持续时间转换为采样点数
        if (durationMs > 0) {
            toneSamplesLeft_ = static_cast<uint32_t>(16) * durationMs;  // 16 samples/ms
        } else {
            toneSamplesLeft_ = 0xFFFFFFFF;  // 无限循环
        }
        looping_ = (durationMs == 0);

        // 启动播放
        amp_.enable();
        playing_ = true;
        pwm_.startWithIT();
    }

    /**
     * @brief 停止播放
     */
    void stop() {
        stopInternal();
    }

    /** @brief 查询是否正在播放 */
    bool isPlaying() const { return playing_; }

    /**
     * @brief 设置音量
     * @param vol 音量值 (0~80)
     * @details 音量直接影响 PWM 占空比的缩放因子。
     *          最大值 80 限制了最大占空比约 80%，保护扬声器。
     *          1W 额定功率扬声器，最大 1.5W:
     *          P = (V_rms)^2 / R，限制 PWM 占空比可限制有效输出功率。
     */
    void setVolume(uint8_t vol) {
        if (vol > MAX_VOLUME) vol = MAX_VOLUME;
        volume_ = vol;
    }

    /** @brief 获取当前音量 */
    uint8_t getVolume() const { return volume_; }

    /**
     * @brief TIM Update 中断回调 — 每 62.5μs (16kHz) 调用一次
     * @details 从缓冲区取一个采样值并设置 PWM CCR。
     *          此函数在中断上下文中执行，必须尽可能快！
     *
     *          PWM 占空比计算：
     *          CCR = sample * 125 / 8 * volume / 100
     *          其中 sample * 125 / 8 是将 8-bit (0~255) 映射到 0~3984
     *          再乘以 volume/100 进行音量缩放
     */
    void onTimerUpdate() {
        if (!playing_) return;

        uint8_t sample;

        if (toneMode_) {
            // 纯音模式：查表生成正弦波
            sample = generateToneSample();
            if (toneSamplesLeft_ != 0xFFFFFFFF) {
                if (toneSamplesLeft_ > 0) {
                    --toneSamplesLeft_;
                } else {
                    stopInternal();
                    return;
                }
            }
        } else {
            // Flash 音频模式：从缓冲区取样本
            if (!bufReady_[bufIndex_]) {
                // 缓冲区未就绪，输出静音
                pwm_.setCompare(0);
                return;
            }
            sample = buffer_[bufIndex_][bufPos_];
            ++bufPos_;

            // 当前缓冲区用完，切换到另一个
            if (bufPos_ >= bufFilled_[bufIndex_]) {
                uint8_t consumed = bufIndex_;
                bufIndex_ ^= 1;  // 切换缓冲区 (0<->1)
                bufPos_ = 0;
                bufReady_[consumed] = false;  // 标记已消费的缓冲区需要重填
                needFill_ = true;             // 标记需要在主循环中填充

                // 如果另一个缓冲区也没准备好且没有剩余数据
                if (!bufReady_[bufIndex_] && remainBytes_ == 0) {
                    if (looping_) {
                        // 循环播放：重置地址
                        currentAddr_ = trackStartAddr_;
                        remainBytes_ = trackLength_;
                    } else {
                        stopInternal();
                        return;
                    }
                }
            }
        }

        // 设置 PWM CCR (含音量缩放)
        // CCR = sample * 125 / 8 → 映射到 0~3984
        // 再 * volume / 100 → 音量缩放
        uint32_t ccr = static_cast<uint32_t>(sample) * 125UL / 8UL;
        ccr = ccr * volume_ / 100;
        pwm_.setCompare(ccr);
    }

    /**
     * @brief 主循环中调用 — 处理缓冲区填充
     * @details 在中断外的主循环中调用，用于从 Flash 读取数据填充空闲缓冲区。
     *          这样避免在中断中执行耗时的 SPI 操作。
     */
    void update() {
        if (!playing_ || toneMode_) return;

        if (needFill_) {
            needFill_ = false;
            // 填充未就绪的缓冲区
            for (uint8_t i = 0; i < 2; ++i) {
                if (!bufReady_[i]) {
                    fillBuffer(i);
                }
            }
        }
    }

private:
    hdl::W25Q64Driver& flash_;  ///< Flash 驱动引用
    hdl::AmpControl& amp_;      ///< 功放控制引用
    hal::Pwm& pwm_;             ///< PWM 输出引用

    // 播放状态
    volatile bool playing_;      ///< 是否正在播放
    bool looping_;               ///< 是否循环播放
    uint8_t volume_;             ///< 音量 (0~80)

    // Flash 读取状态
    uint32_t trackStartAddr_;    ///< 当前轨道起始地址
    uint32_t trackLength_;       ///< 当前轨道总长度
    uint32_t currentAddr_;       ///< 当前读取地址
    uint32_t remainBytes_;       ///< 剩余未读字节数

    // 双缓冲区
    uint8_t buffer_[2][BUFFER_SIZE];       ///< 双缓冲区
    volatile uint8_t bufIndex_;            ///< 当前播放的缓冲区索引 (0 或 1)
    volatile uint16_t bufPos_;             ///< 当前缓冲区内播放位置
    volatile bool bufReady_[2];            ///< 缓冲区就绪标志
    uint16_t bufFilled_[2];               ///< 每个缓冲区实际填充的字节数

    // 纯音模式
    bool toneMode_;              ///< 是否为纯音模式
    uint16_t tonePhase_;         ///< 当前相位 (0 ~ tonePeriodSamples_-1)
    uint16_t tonePeriodSamples_; ///< 一个周期的采样点数
    uint32_t toneSamplesLeft_;   ///< 剩余采样点数

    // 需要在主循环中填充缓冲区标志 (必须在 toneSamplesLeft_ 之后声明以匹配初始化顺序)
    volatile bool needFill_;               ///< 需要填充缓冲区标志

    /**
     * @brief 填充指定缓冲区
     * @param idx 缓冲区索引 (0 或 1)
     */
    void fillBuffer(uint8_t idx) {
        if (remainBytes_ == 0) {
            if (looping_) {
                currentAddr_ = trackStartAddr_;
                remainBytes_ = trackLength_;
            } else {
                bufReady_[idx] = false;
                bufFilled_[idx] = 0;
                return;
            }
        }

        uint16_t toRead = (remainBytes_ >= BUFFER_SIZE)
                          ? BUFFER_SIZE
                          : static_cast<uint16_t>(remainBytes_);
        flash_.readData(currentAddr_, buffer_[idx], toRead);
        currentAddr_ += toRead;
        remainBytes_ -= toRead;
        bufFilled_[idx] = toRead;
        bufReady_[idx] = true;
    }

    /**
     * @brief 生成纯音采样值（正弦波查表）
     * @return 8-bit unsigned 采样值
     * @details 使用简单三角函数近似正弦波：
     *          phase 从 0 到 tonePeriodSamples_-1
     *          将 phase 映射到 0~2π，计算 sin 值
     *          输出 128 + 127*sin(2π * phase / period)
     *
     *          为避免在中断中使用浮点 sin 函数，
     *          使用分段线性近似正弦波(更快)：
     *          0~T/4:     从 128 线性上升到 255
     *          T/4~T/2:   从 255 线性下降到 128
     *          T/2~3T/4:  从 128 线性下降到 1
     *          3T/4~T:    从 1 线性上升到 128
     */
    uint8_t generateToneSample() {
        uint16_t period = tonePeriodSamples_;
        uint16_t phase = tonePhase_;

        // 分段线性近似正弦波
        uint8_t sample;
        uint16_t quarter = period / 4;
        if (quarter == 0) quarter = 1;

        if (phase < quarter) {
            // 第一象限：128 → 255
            sample = 128 + static_cast<uint8_t>(
                static_cast<uint32_t>(127) * phase / quarter);
        } else if (phase < 2 * quarter) {
            // 第二象限：255 → 128
            sample = 255 - static_cast<uint8_t>(
                static_cast<uint32_t>(127) * (phase - quarter) / quarter);
        } else if (phase < 3 * quarter) {
            // 第三象限：128 → 1
            sample = 128 - static_cast<uint8_t>(
                static_cast<uint32_t>(127) * (phase - 2 * quarter) / quarter);
        } else {
            // 第四象限：1 → 128
            sample = 1 + static_cast<uint8_t>(
                static_cast<uint32_t>(127) * (phase - 3 * quarter) / quarter);
        }

        // 推进相位
        ++tonePhase_;
        if (tonePhase_ >= period) {
            tonePhase_ = 0;
        }

        return sample;
    }

    /**
     * @brief 内部停止播放
     */
    void stopInternal() {
        playing_ = false;
        pwm_.stopWithIT();
        pwm_.setCompare(0);
        amp_.disable();
        bufReady_[0] = false;
        bufReady_[1] = false;
        bufPos_ = 0;
        bufIndex_ = 0;
        needFill_ = false;
    }
};

} // namespace fml

#endif // AUDIO_PLAYER_HPP
