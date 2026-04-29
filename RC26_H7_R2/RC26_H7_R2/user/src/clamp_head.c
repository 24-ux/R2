#include "clamp_head.h"

typedef enum {
    CLAMP_HEAD_IDLE,
    CLAMP_HEAD_WAIT_CLOSE,
    CLAMP_HEAD_WAIT_DELAY,
    CLAMP_HEAD_DONE
} ClampHeadState;

static ClampHeadState clamp_head_state = CLAMP_HEAD_IDLE;
static uint32_t clamp_close_start_time = 0;
static GPIO_PinState last_switch_state = GPIO_PIN_RESET;

void clamp_head_init(void)
{
    clamp_head_state = CLAMP_HEAD_IDLE;
    last_switch_state = GPIO_PIN_RESET;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
}

void clamp_head_auto_process(void)
{
    GPIO_PinState switch_state;
    uint32_t current_time;

    switch (clamp_head_state)
    {
        case CLAMP_HEAD_IDLE:
            switch_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
            if (switch_state == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
                clamp_head_state = CLAMP_HEAD_WAIT_CLOSE;
                clamp_close_start_time = HAL_GetTick();
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            }
            break;

        case CLAMP_HEAD_WAIT_CLOSE:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1100);
            switch_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
            if (switch_state == GPIO_PIN_RESET)
            {
                clamp_head_state = CLAMP_HEAD_IDLE;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
            }
            else
            {
                current_time = HAL_GetTick();
                if ((current_time - clamp_close_start_time) >= 200)
                {
                    clamp_head_state = CLAMP_HEAD_WAIT_DELAY;
                }
            }
            break;

        case CLAMP_HEAD_WAIT_DELAY:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);
            switch_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
            if (switch_state == GPIO_PIN_RESET)
            {
                clamp_head_state = CLAMP_HEAD_IDLE;
            }
            break;

        case CLAMP_HEAD_DONE:
            switch_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
            if (switch_state == GPIO_PIN_RESET)
            {
                clamp_head_state = CLAMP_HEAD_IDLE;
            }
            break;

        default:
            clamp_head_state = CLAMP_HEAD_IDLE;
            break;
    }

    last_switch_state = switch_state;
}
