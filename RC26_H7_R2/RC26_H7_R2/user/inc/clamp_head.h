#ifndef __CLAMP_HEAD_H__
#define __CLAMP_HEAD_H__

#include "main.h"
#include "tim.h"

typedef enum {
    CLAMP_HEAD_IDLE,
    CLAMP_HEAD_WAIT_CLOSE
} ClampHeadState;

void clamp_head_init(void);
void clamp_head_auto_process(void);

#endif
