/**
 * @file    user_bridge.h
 * @brief   用户 C++ 代码的 C 语言接口桥接头文件
 * @details 由于 CubeMX 生成的 main.c 是 C 文件，无法直接包含 C++ 头文件。
 *          通过此桥接层提供 C 兼容的函数声明，在 user_bridge.cpp 中实现。
 */
#ifndef USER_BRIDGE_H
#define USER_BRIDGE_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 用户应用初始化 (在 main 的 while(1) 之前调用)
 */
void User_App_Init(void);

/**
 * @brief 用户应用主循环 (在 main 的 while(1) 中调用)
 */
void User_App_Loop(void);

/**
 * @brief TIM 周期溢出中断回调
 * @param htim TIM 句柄
 */
void User_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

/**
 * @brief UART 接收事件回调 (DMA 空闲中断)
 * @param huart UART 句柄
 * @param Size  接收到的数据长度
 */
void User_UART_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif // USER_BRIDGE_H
