#include "clamp_head.h"

static ClampHeadState clamp_head_state = CLAMP_HEAD_IDLE;
static uint32_t clamp_close_start_time = 0;

void clamp_head_init(void)
{
    clamp_head_state = CLAMP_HEAD_IDLE;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
}

void clamp_head_auto_process(void)
{
    GPIO_PinState switch_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
    uint32_t current_time = HAL_GetTick();

    switch (clamp_head_state)
    {
        case CLAMP_HEAD_IDLE:
            if (switch_state == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
                clamp_close_start_time = current_time;
                clamp_head_state = CLAMP_HEAD_WAIT_CLOSE;
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            }
            break;

        case CLAMP_HEAD_WAIT_CLOSE:
            if (switch_state == GPIO_PIN_RESET)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
                clamp_head_state = CLAMP_HEAD_IDLE;
            }
            else if ((current_time - clamp_close_start_time) >= 200)
            {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);
                clamp_head_state = CLAMP_HEAD_WAIT_DELAY;
            }
            break;

        case CLAMP_HEAD_WAIT_DELAY:
            if (switch_state == GPIO_PIN_RESET)
            {
                clamp_head_state = CLAMP_HEAD_IDLE;
            }
            break;
    }
}
