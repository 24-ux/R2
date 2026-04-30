#ifndef __BSP_CAN_TIM_H__
#define __BSP_CAN_TIM_H__

#include "main.h"

/* 发送标志位 */
extern volatile uint8_t can1_send_flag;
extern volatile uint8_t can2_send_flag;
extern volatile uint8_t can3_send_flag;

void BSP_CAN_Tim_Init(void);

#endif