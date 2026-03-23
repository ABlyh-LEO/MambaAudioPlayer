/**
 * @file    app_main.hpp
 * @brief   应用层主逻辑
 * @details 系统状态机管理、各模块初始化和主循环逻辑。
 *
 *          系统状态机：
 *          ┌──────────────────────────────────────────┐
 *          │           STATE_NORMAL                   │
 *          │  电池正常, 绿灯常亮, 心跳灯闪烁         │
 *          └────────┬───────────────────┬─────────────┘
 *                   │ 低电量            │ UART传输请求
 *          ┌────────▼───────────┐  ┌───▼──────────────┐
 *          │ STATE_LOW_BATTERY  │  │ STATE_TRANSFER   │
 *          │ 红灯闪烁+音频播放 │  │ UART音频传输中   │
 *          └────────┬───────────┘  └──────────────────┘
 *                   │ 按键按下
 *          ┌────────▼───────────┐
 *          │   STATE_MUTED      │
 *          │ 60s静音, 红灯常亮  │
 *          └────────────────────┘
 */
#ifndef APP_MAIN_HPP
#define APP_MAIN_HPP

#include "main.h"
#include "HAL/hal_gpio.hpp"
#include "HAL/hal_adc.hpp"
#include "HAL/hal_spi.hpp"
#include "HAL/hal_i2c.hpp"
#include "HAL/hal_uart.hpp"
#include "HAL/hal_pwm.hpp"
#include "HDL/w25q64_driver.hpp"
#include "HDL/ws2812_driver.hpp"
#include "HDL/battery_rack.hpp"
#include "HDL/amp_control.hpp"
#include "FML/audio_player.hpp"
#include "FML/battery_monitor.hpp"
#include "FML/led_indicator.hpp"
#include "FML/uart_transfer.hpp"

/// 外部 HAL 句柄声明 (在 main.c 中由 CubeMX 生成)
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

namespace apl {

/// 系统状态枚举
enum class SystemState {
    NORMAL,           ///< 正常监控
    LOW_BATTERY,      ///< 低电量报警
    MUTED,            ///< 按键静音中 (60s)
    AUDIO_TRANSFER    ///< 音频传输中
};

/**
 * @class AppMain
 * @brief 应用主控制类
 * @details 管理所有软件模块的初始化、主循环逻辑和状态切换。
 *          使用单例模式，全局唯一实例。
 */
class AppMain {
public:
    /// 按键静音持续时间 (ms)
    static constexpr uint32_t MUTE_DURATION_MS = 60000;

    /// 按键消抖时间 (ms)
    static constexpr uint32_t DEBOUNCE_MS = 50;

    /// 电池状态检测间隔 (ms)
    static constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 1000;

    /// UART 日志输出间隔 (ms)
    static constexpr uint32_t LOG_INTERVAL_MS = 5000;

    /**
     * @brief 获取全局单例
     * @return AppMain 单例引用
     */
    static AppMain& getInstance() {
        static AppMain instance;
        return instance;
    }

    /**
     * @brief 系统初始化 (在 main 函数的 while(1) 之前调用)
     * @details 初始化顺序：
     *          1. HAL 层对象绑定硬件句柄
     *          2. HDL 层驱动初始化
     *          3. FML 层模块初始化
     *          4. 启动各外设
     */
    void init() {
        // 日志输出启动信息
        uart_.log("\r\n=============================\r\n");
        uart_.log("[MAMBA] System starting...\r\n");
        uart_.log("[MAMBA] MCU: STM32G030F6P6 @64MHz\r\n");

        // 初始化 LED 指示灯
        ledIndicator_.init();
        uart_.log("[MAMBA] LED init OK\r\n");

        // 初始化 SPI Flash
        if (flash_.init()) {
            uart_.log("[MAMBA] W25Q64 init OK (JEDEC ID matched)\r\n");
            flashOk_ = true;
        } else {
            uart_.log("[MAMBA] W25Q64 init FAILED!\r\n");
            flashOk_ = false;
        }

        // 初始化电池监测
        batteryMonitor_.init();
        uart_.log("[MAMBA] Battery monitor init OK\r\n");

        if (batteryMonitor_.isI2CAvailable()) {
            uart_.log("[MAMBA] I2C battery rack detected!\r\n");
        } else {
            uart_.log("[MAMBA] I2C battery rack NOT connected, using ADC\r\n");
        }

        // 初始化 UART 传输
        uartTransfer_.init();
        uart_.log("[MAMBA] UART transfer init OK\r\n");

        // 初始化功放为关断状态
        amp_.disable();

        uart_.log("[MAMBA] System ready.\r\n");
        uart_.log("=============================\r\n");

        // 记录启动时间
        lastBatteryCheckTick_ = HAL_GetTick();
        lastLogTick_ = HAL_GetTick();
    }

    /**
     * @brief 主循环 (在 while(1) 中调用)
     * @details 执行顺序：
     *          1. 获取当前 tick
     *          2. 更新电池监测
     *          3. 更新按键状态
     *          4. 处理状态机逻辑
     *          5. 更新 LED 指示灯
     *          6. 处理音频播放缓冲区填充
     *          7. 处理 UART 数据传输
     *          8. 定期输出日志
     */
    void loop() {
        uint32_t tick = HAL_GetTick();

        // 更新各模块
        batteryMonitor_.update(tick);
        ledIndicator_.update(tick);
        audioPlayer_.update();

        // 同步系统状态到 UART 传输模块 (供 QUERY_STATUS 命令使用)
        uartTransfer_.updateStatus(
            batteryMonitor_.getVoltage(),
            batteryMonitor_.getBatteryPercent(),
            batteryMonitor_.isI2CAvailable(),
            static_cast<uint8_t>(state_)  // 枚举值 0~3 直接对应协议定义
        );
        uartTransfer_.processReceive();

        // 处理按键
        processButton(tick);

        // 状态机处理
        processStateMachine(tick);

        // 移除定期的 printStatus();，可通过上位机发送 QUERY_STATUS 查询状态
    }

    /**
     * @brief TIM1 Update 中断回调
     * @details 每 62.5μs (16kHz) 调用一次
     *          由 HAL_TIM_PeriodElapsedCallback 调用
     */
    void onTimerUpdate() {
        audioPlayer_.onTimerUpdate();
    }

    /**
     * @brief UART 接收完成回调
     * @param size 接收到的数据长度
     */
    void onUartReceiveComplete(uint16_t size) {
        uartTransfer_.onReceiveComplete(size);
    }

private:
    // ========== HAL 层对象 ==========
    hal::Gpio gpioLed_        {LED_DEBUG_GPIO_Port, LED_DEBUG_Pin};       ///< 心跳灯 PA7
    hal::Gpio gpioBtn_        {BTN_FUNC_GPIO_Port, BTN_FUNC_Pin};        ///< 按键 PC14
    hal::Gpio gpioFlashCs_    {FLASH_CS_GPIO_Port, FLASH_CS_Pin};        ///< Flash CS PA4
    hal::Gpio gpioAmpShdn_    {AMP_SHDN_GPIO_Port, AMP_SHDN_Pin};       ///< 功放 SHDN PA3
    hal::Adc  adc_            {&hadc1};                                   ///< ADC
    hal::Spi  spi_            {&hspi1};                                   ///< SPI
    hal::I2c  i2c_            {&hi2c2};                                   ///< I2C
    hal::Uart uart_           {&huart1};                                  ///< UART
    hal::Pwm  pwm_            {&htim1, TIM_CHANNEL_1};                   ///< PWM (TIM1_CH1)

    // ========== HDL 层对象 ==========
    hdl::W25Q64Driver  flash_       {spi_, gpioFlashCs_};   ///< SPI Flash 驱动
    hdl::WS2812Driver  ws2812_      {WS2812_DIN_GPIO_Port, WS2812_DIN_Pin}; ///< WS2812 驱动
    hdl::BatteryRack   batteryRack_ {i2c_};                  ///< 电池架驱动
    hdl::AmpControl    amp_         {gpioAmpShdn_};          ///< 功放控制

    // ========== FML 层对象 ==========
    fml::AudioPlayer     audioPlayer_     {flash_, amp_, pwm_};          ///< 音频播放器
    fml::BatteryMonitor  batteryMonitor_  {adc_, batteryRack_};          ///< 电池监测
    fml::LedIndicator    ledIndicator_    {gpioLed_, ws2812_};           ///< LED 指示灯
    fml::UartTransfer    uartTransfer_    {uart_, flash_};               ///< UART 传输

    // ========== 状态变量 ==========
    SystemState state_          = SystemState::NORMAL;  ///< 系统状态
    bool flashOk_               = false;                ///< Flash 初始化成功标志
    uint32_t muteStartTick_     = 0;                    ///< 静音开始时间
    uint32_t lastBatteryCheckTick_ = 0;                 ///< 上次电池检测时间
    uint32_t lastLogTick_       = 0;                    ///< 上次日志输出时间
    bool lastBtnState_          = true;                 ///< 上次按键状态 (未按下=高)
    uint32_t btnPressTick_      = 0;                    ///< 按键按下时间 (消抖用)
    bool alarmAudioStarted_     = false;                ///< 报警音频是否已启动

    /** @brief 私有构造函数 (单例) */
    AppMain() = default;

    /**
     * @brief 处理按键输入
     * @param tick 当前系统时间
     * @details 按键低电平有效 (PC14, 内部上拉)
     *          外部 100nF 电容硬件消抖 + 软件 50ms 消抖
     */
    void processButton(uint32_t tick) {
        bool btnState = gpioBtn_.read();  // true=未按下(高), false=按下(低)

        // 检测下降沿 (按下)
        if (lastBtnState_ && !btnState) {
            btnPressTick_ = tick;
        }
        // 检测上升沿 (释放) 并确认消抖
        if (!lastBtnState_ && btnState) {
            if (tick - btnPressTick_ >= DEBOUNCE_MS) {
                onButtonPressed(tick);
            }
        }
        lastBtnState_ = btnState;
    }

    /**
     * @brief 按键按下事件处理
     * @param tick 当前时间
     * @details 按下按键后 60s 内不进行音频播放
     */
    void onButtonPressed(uint32_t tick) {
        if (state_ == SystemState::LOW_BATTERY) {
            // 进入静音模式
            audioPlayer_.stop();
            muteStartTick_ = tick;
            state_ = SystemState::MUTED;
            ledIndicator_.setState(fml::LedState::MUTED);
            alarmAudioStarted_ = false;
        }
    }

    /**
     * @brief 状态机处理
     * @param tick 当前时间
     */
    void processStateMachine(uint32_t tick) {
        // UART 传输优先级最高
        if (uartTransfer_.isTransferring()) {
            if (state_ != SystemState::AUDIO_TRANSFER) {
                audioPlayer_.stop();
                state_ = SystemState::AUDIO_TRANSFER;
            }
            return;
        } else if (state_ == SystemState::AUDIO_TRANSFER) {
            state_ = SystemState::NORMAL;
            alarmAudioStarted_ = false;
        }

        // 电池状态检测
        bool lowBattery = batteryMonitor_.isLowBattery();

        switch (state_) {
            case SystemState::NORMAL:
                if (lowBattery) {
                    state_ = SystemState::LOW_BATTERY;
                    ledIndicator_.setState(fml::LedState::LOW_BATTERY);
                    alarmAudioStarted_ = false;
                } else {
                    ledIndicator_.setState(fml::LedState::NORMAL);
                }
                break;

            case SystemState::LOW_BATTERY:
                if (!lowBattery) {
                    // 电量恢复
                    audioPlayer_.stop();
                    state_ = SystemState::NORMAL;
                    ledIndicator_.setState(fml::LedState::NORMAL);
                    alarmAudioStarted_ = false;
                } else if (!alarmAudioStarted_) {
                    // 启动报警音频
                    startAlarmAudio();
                    alarmAudioStarted_ = true;
                } else if (!audioPlayer_.isPlaying()) {
                    // 音频播完后重新启动（循环播放逻辑备份）
                    startAlarmAudio();
                }
                break;

            case SystemState::MUTED:
                // 检查静音时间是否到期
                if (tick - muteStartTick_ >= MUTE_DURATION_MS) {
                    if (lowBattery) {
                        state_ = SystemState::LOW_BATTERY;
                        ledIndicator_.setState(fml::LedState::LOW_BATTERY);
                        alarmAudioStarted_ = false;
                    } else {
                        state_ = SystemState::NORMAL;
                        ledIndicator_.setState(fml::LedState::NORMAL);
                    }
                }
                // 静音期间也检查电池是否恢复
                if (!lowBattery) {
                    state_ = SystemState::NORMAL;
                    ledIndicator_.setState(fml::LedState::NORMAL);
                }
                break;

            case SystemState::AUDIO_TRANSFER:
                // 已在上面处理
                break;
        }
    }

    /**
     * @brief 启动报警音频
     * @details 优先播放 Flash 中第一段音频 (循环播放)。
     *          如果第一段音频无效，则播放 440Hz 纯音。
     */
    void startAlarmAudio() {
        if (flashOk_ && audioPlayer_.playTrack(0, true)) {
            // Playing alarm track 0
        } else {
            // 第一段音频无效，播放 440Hz 报警音
            audioPlayer_.playTone(440, 0);  // 0 = 无限循环
        }
    }

};

} // namespace apl

#endif // APP_MAIN_HPP
