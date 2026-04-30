#include "bsp_can_tim.h"
#include "tim.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"

/* 发送标志位 */
volatile uint8_t can1_send_flag = 0;
volatile uint8_t can2_send_flag = 0;
volatile uint8_t can3_send_flag = 0;

void BSP_CAN_Tim_Init(void)
{
    /* 启动三个定时器 */
    HAL_TIM_Base_Start_IT(&htim3);  /* TIM3 → FDCAN1 */
    HAL_TIM_Base_Start_IT(&htim4);  /* TIM4 → FDCAN2 */
    HAL_TIM_Base_Start_IT(&htim5);  /* TIM5 → FDCAN3 */
}
