/**
 * @file    user_bridge.cpp
 * @brief   用户 C++ 代码的 C 语言接口桥接实现
 * @details 实现 user_bridge.h 中声明的 C 兼容函数，
 *          内部调用 C++ 应用层的单例方法。
 */

#include "APL/app_main.hpp"
#include "user_bridge.h"

extern "C" {

/**
 * @brief 用户应用初始化
 */
void User_App_Init(void) {
    apl::AppMain::getInstance().init();
}

/**
 * @brief 用户应用主循环
 */
void User_App_Loop(void) {
    apl::AppMain::getInstance().loop();
}

/**
 * @brief TIM 周期溢出中断回调
 */
void User_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM1) {
        apl::AppMain::getInstance().onTimerUpdate();
    }
}

/**
 * @brief UART 接收事件回调
 */
void User_UART_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        apl::AppMain::getInstance().onUartReceiveComplete(Size);
    }
}

} // extern "C"
