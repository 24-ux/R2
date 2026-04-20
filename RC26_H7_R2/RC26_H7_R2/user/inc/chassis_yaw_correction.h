#ifndef __CHASSIS_YAW_CORRECTION_H__
#define __CHASSIS_YAW_CORRECTION_H__

#include "chassis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 航向矫正（陀螺仪 + 双环PID）模块
 * 设计目标：当“有平移、无人工旋转”时，自动输出旋转补偿以保持航向。
 * 说明：模块不直接读取 gyro[] 全局数组，由调用方传入 gyro_z(rad/s)。
 */

void chassis_yaw_corr_reset(void);

/* 对 chassis->param.Vx_in 叠加旋转补偿（Vx_in 为旋转通道） */
void chassis_yaw_corr_apply(Chassis_Module *chassis, float gyro_z_radps);

#ifdef __cplusplus
}
#endif

#endif

