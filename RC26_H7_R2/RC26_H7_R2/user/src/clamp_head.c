#include "clamp_head.h"

static ClampHeadState clamp_head_state = CLAMP_HEAD_IDLE;
static uint32_t clamp_close_start_time = 0;
uint8_t switch_state;
void clamp_head_init(void)
{
    clamp_head_state = CLAMP_HEAD_IDLE;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1150);  // 空闲：舵机中间
}

void clamp_head_auto_process(void)
{
    switch_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
    uint32_t current_time = HAL_GetTick();

    switch (clamp_head_state)
    {
        case CLAMP_HEAD_IDLE:
            // 空闲：舵机中间(1100) + 夹爪松开
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1150);
            
            // 放东西 → PE9低电平
            if (switch_state == GPIO_PIN_RESET)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);  // 第一步：夹爪夹紧
                clamp_close_start_time = current_time;                // 开始计时200ms
                clamp_head_state = CLAMP_HEAD_WAIT_CLOSE;             // 进入等待夹紧
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // 没东西：夹爪松开
            }
            break;

        case CLAMP_HEAD_WAIT_CLOSE:
            // ===================== 这里按你要求修改 =====================
            // 随时取消：拿走东西 → 夹爪松开 + 舵机保持直立
            if (switch_state == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); // 夹爪立刻松开
                // 舵机保持直立，不回中间！！！
                clamp_head_state = CLAMP_HEAD_IDLE;
                break;
            }

            // 等待200ms确保夹紧完成
            if ((current_time - clamp_close_start_time) >= 200)
            {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);  // 第二步：舵机转到直立
            }
            break;
    }
}