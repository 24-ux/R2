#include "chassis.h"
#include "Motion_Task.h"
#include "lift.h"
#include "master_control.h"
#include <math.h>

Chassis_Module Chassis;


// 底盘电机
DJI_MotorModule chassis_motor1;  // 左前
DJI_MotorModule chassis_motor2;  // 右前
DJI_MotorModule chassis_motor3;  // 左后
DJI_MotorModule chassis_motor4;  // 右后

// 导轮电机
DJI_MotorModule guide_motor1;  // 左
DJI_MotorModule guide_motor2;  // 右
 
/* 来自 Can_Task 的 BMI088 角速度数据，gyro[2] 为 Z 轴角速度 */
extern float gyro[3];

/* 航向保持参数：用于抑制重心偏导致的直行“慢慢拧头” */
#define CHASSIS_YAW_HOLD_KP              (10.0f)   /* 比例：航向误差越大，纠偏越强；过大易振荡 */
#define CHASSIS_YAW_HOLD_KD              (1.8f)   /* 微分：用陀螺 Z 角速度阻尼，抑制过冲 */
#define CHASSIS_YAW_HOLD_INT_LIMIT       (1.8f)   /* 积分限幅：防止积分项过大导致“顶死” */
#define CHASSIS_YAW_HOLD_OUT_LIMIT       (20.0f)  /* 输出限幅：叠加到 Vx_in 的最大纠偏量 */
#define CHASSIS_YAW_HOLD_ROT_DEADZONE    (2.0f)   /* 旋转死区：|Vx_in| 超过则认为手在主动转向，关闭保持 */
#define CHASSIS_YAW_HOLD_MOVE_DEADZONE   (0.2f)   /* 平移死区：|Vy_in|/|Vw_in| 低于则认为停车，不保持 */

static float chassis_yaw_est = 0.0f;           /* 当前估计航向(rad)，由 gyro_z 积分得到 */
static float chassis_yaw_target = 0.0f;        /* 要保持的目标航向(rad)，进入直行时锁定 */
static float chassis_yaw_i = 0.0f;             /* 航向误差积分项（带限幅） */
static uint8_t chassis_yaw_hold_active = 0U;   /* 1=已进入航向保持；0=未保持或已退出 */
static uint32_t chassis_yaw_last_tick = 0U;    /* 上次计算时刻(ms)，用于求 dt */

/* 将弧度折叠到 (-pi, pi]：航向/误差在圆周上连续，不 wrap 时 +179° 与 -179° 会差成 358° 的假误差 */
static float wrap_pi(float a)
{
    const float pi = 3.1415926f;
    const float two_pi = 6.2831853f;
    /* 反复减/加 2pi，直到落入主值区间 */
    while (a > pi) a -= two_pi;
    while (a < -pi) a += two_pi;
    return a;
}

static float clampf_local(float v, float min_v, float max_v)
{
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

static void chassis_apply_yaw_hold(Chassis_Module *chassis)
{
    uint32_t now_tick = HAL_GetTick();
    float dt = 0.003f; /* 默认按任务周期 3ms */
    float yaw_err;
    float yaw_hold_out;
    float gyro_z;
    uint8_t has_rot_cmd;
    uint8_t has_move_cmd;

    if (chassis_yaw_last_tick != 0U && now_tick >= chassis_yaw_last_tick)
    {
        dt = (float)(now_tick - chassis_yaw_last_tick) * 0.001f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.02f)  dt = 0.02f;
    }
    chassis_yaw_last_tick = now_tick;

    gyro_z = gyro[2];
    chassis_yaw_est = wrap_pi(chassis_yaw_est + gyro_z * dt);

    has_rot_cmd = (fabsf(chassis->param.Vx_in) > CHASSIS_YAW_HOLD_ROT_DEADZONE) ? 1U : 0U;
    has_move_cmd = (fabsf(chassis->param.Vy_in) > CHASSIS_YAW_HOLD_MOVE_DEADZONE ||
                    fabsf(chassis->param.Vw_in) > CHASSIS_YAW_HOLD_MOVE_DEADZONE) ? 1U : 0U;

    /* 手动旋转时不做保持；停止移动时复位保持状态 */
    if (has_rot_cmd != 0U || has_move_cmd == 0U)
    {
        chassis_yaw_hold_active = 0U;
        chassis_yaw_i = 0.0f;
        chassis_yaw_target = chassis_yaw_est;
        return;
    }

    if (chassis_yaw_hold_active == 0U)
    {
        chassis_yaw_hold_active = 1U;
        chassis_yaw_target = chassis_yaw_est;
        chassis_yaw_i = 0.0f;
    }

    yaw_err = wrap_pi(chassis_yaw_target - chassis_yaw_est);
    chassis_yaw_i = clampf_local(chassis_yaw_i + yaw_err * dt, -CHASSIS_YAW_HOLD_INT_LIMIT, CHASSIS_YAW_HOLD_INT_LIMIT);
    yaw_hold_out = CHASSIS_YAW_HOLD_KP * yaw_err + 0.4f * chassis_yaw_i - CHASSIS_YAW_HOLD_KD * gyro_z;
    yaw_hold_out = clampf_local(yaw_hold_out, -CHASSIS_YAW_HOLD_OUT_LIMIT, CHASSIS_YAW_HOLD_OUT_LIMIT);

    /* 叠加到旋转通道，抵消偏航 */
    chassis->param.Vx_in += yaw_hold_out;
}



/* 当前底盘指令缓存（用于将位定义转换为速度输入） */
static master_chassis_cmd_t g_master_chassis_cmd;

static void chassis_decode_master_cmd(uint8_t action_byte0, uint8_t action_byte1)
{
    g_master_chassis_cmd.speed_level = (chassis_speed_level_t)((action_byte0 >> 6) & 0x03U);
    g_master_chassis_cmd.move_dir = (chassis_move_dir_t)((action_byte0 >> 3) & 0x07U);
    g_master_chassis_cmd.rot_dir = (chassis_rot_dir_t)((action_byte0 >> 1) & 0x03U);
    g_master_chassis_cmd.flexible_extend = (uint8_t)(action_byte0 & 0x01U);
    g_master_chassis_cmd.reserved_byte1 = action_byte1;
}

static uint16_t chassis_get_speed_amp(chassis_speed_level_t level)
{
    const uint16_t span = (uint16_t)(CH2_HIGH - CH2_MID); /* 800 */

    switch (level)
    {
        case CHASSIS_SPEED_LOW:
            return (uint16_t)(span / 10U);        /* 1/10 */
        case CHASSIS_SPEED_NORMAL:
            return (uint16_t)(span / 2U);         /* 1/2 */
        case CHASSIS_SPEED_HIGH:
            return span;                          /* 1 */
        case CHASSIS_SPEED_SUPER_HIGH:
            return span * 1.5f;                      /* 2 */
        default:
            return 0U;
    }
}
static void chassis_apply_master_motion(void)
{
    uint16_t amp = chassis_get_speed_amp(g_master_chassis_cmd.speed_level);
    float trans = ((float)amp / (float)(CH2_HIGH - CH2_MID)) * 100.0f;
    float rot = (40.2814f / 4.0f) * (((float)amp / (float)(CH4_HIGH - CH4_MID)) * 5.0f);

    /* master 模式下直接写底盘输入，不再复用 RC 通道值 */
    Chassis.param.Accel = 100.0f;
    Chassis.param.Vw_in = 0.0f;
    Chassis.param.Vy_in = 0.0f;
    Chassis.param.Vx_in = 0.0f;

    switch (g_master_chassis_cmd.move_dir)
    {
        case CHASSIS_DIR_FORWARD:
            Chassis.param.Vy_in = trans;
            break;
        case CHASSIS_DIR_BACKWARD:
            Chassis.param.Vy_in = -trans;
            break;
        case CHASSIS_DIR_LEFT:
            Chassis.param.Vw_in = -trans;
            break;
        case CHASSIS_DIR_RIGHT:
            Chassis.param.Vw_in = trans;
            break;
        case CHASSIS_DIR_NONE:
        default:
            break;
    }

    switch (g_master_chassis_cmd.rot_dir)
    {
        case CHASSIS_ROT_LEFT:
            Chassis.param.Vx_in = -rot;
            break;
        case CHASSIS_ROT_RIGHT:
            Chassis.param.Vx_in = rot;
            break;
        case CHASSIS_ROT_NONE:
        case CHASSIS_ROT_RESERVED:
        default:
            break;
    }
}



float chassis_motor1_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.20f,1,500.0f,10000.0f};     //KP,KI,KD,DEADBAND,LIMITINTEGRAL,LIMITOUTPUT
float chassis_motor2_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.20f,1,500.0f,10000.0f};
float chassis_motor3_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.20f,1,500.0f,10000.0f};
float chassis_motor4_pid_param[PID_PARAMETER_NUM] = {2.5f,0.05f,0.20f,1,500.0f,10000.0f};

float guide_motor1_pid_param[PID_PARAMETER_NUM] = {3.0f,0.1f,0.2f,1,500.0f,10000.0f};
float guide_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

/**
  * @brief 底盘运行逻辑
  */
void manual_chassis_function(void)
{
    static MasterLevelGate master_chassis_flex_gate = {0U, 0U};

    /* 主控模式：按上位机动作字节解码并写入底盘输入 */
    if (control_mode == master_control)
    {
        chassis_decode_master_cmd(master_chassis_action_bits_0, master_chassis_action_bits_1);
        chassis_apply_master_motion();
    }



    if (control_mode == master_control)
    {
        uint8_t flex_level = (g_master_chassis_cmd.flexible_extend != 0U) ? 1U : 0U;

        if (master_level_gate_on_change(&master_chassis_flex_gate, flex_level) != 0U)
        {
            flex_cmd = flex_level ? FLEX_CMD_EXTEND : FLEX_CMD_RETRACT;
        }
        else
        {
            flex_cmd = FLEX_CMD_NONE;
        }
    }
    else if(control_mode == remote_control)
    {
        flexible_motor_update_command(RCctrl.CH5);
    }
flexible_motor_state_machine_step();

///////////////////////////////////////////////////////////////////////


	Chassis.Chassis_Calc(&Chassis);

	chassis_motor1.PID_Calculate(&chassis_motor1, 50*Chassis.param.V_out[0]);
	chassis_motor2.PID_Calculate(&chassis_motor2, 50*Chassis.param.V_out[1]);
	chassis_motor3.PID_Calculate(&chassis_motor3, 50*Chassis.param.V_out[2]);
	chassis_motor4.PID_Calculate(&chassis_motor4, 50*Chassis.param.V_out[3]);
	
	guide_motor1.PID_Calculate(&guide_motor1, 200*Chassis.param.V_out[0]);
	guide_motor2.PID_Calculate(&guide_motor2, 200*Chassis.param.V_out[1]);

	flexible_motor1.PID_Calculate(&flexible_motor1,flexible_motor_PID_input);
	flexible_motor2.PID_Calculate(&flexible_motor2,-flexible_motor_PID_input);			

	DJIset_motor_data(&hfdcan1, 0X200, chassis_motor1.pid_spd.Output, chassis_motor2.pid_spd.Output,chassis_motor3.pid_spd.Output,chassis_motor4.pid_spd.Output);
	DJIset_motor_data(&hfdcan2, 0X200, guide_motor1.pid_spd.Output, guide_motor2.pid_spd.Output,flexible_motor1.pid_spd.Output,flexible_motor2.pid_spd.Output);
		
//	DJIset_motor_data(&hfdcan1, 0X200,0,0,0,0);
//	DJIset_motor_data(&hfdcan2, 0X200,0,0,0,0);

}


void Chassis_Calc(Chassis_Module *chassis)
{
		
    // 仅遥控模式从 RC 通道读取；master 模式输入由 chassis_apply_master_motion 直接给定
    if (control_mode == remote_control && remote_mode == chassis_mode) {
        chassis->param.Accel = ACCEL;
        chassis->param.Vw_in = LR_TRANSLATION;
        chassis->param.Vy_in = FB_TRANSLATION;
        chassis->param.Vx_in = ROTATION;
    }

    chassis_apply_yaw_hold(chassis);
    
    chassis->param.V_out[0] = chassis->param.Vx_in + chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[1] = chassis->param.Vx_in - chassis->param.Vy_in + chassis->param.Vw_in;
    chassis->param.V_out[2] = chassis->param.Vx_in + chassis->param.Vy_in - chassis->param.Vw_in;
    chassis->param.V_out[3] = chassis->param.Vx_in - chassis->param.Vy_in - chassis->param.Vw_in;
		
		    // ==============================
    // 【前进后退双向补偿】
    // 前进：左轮慢 → 给左轮加力
    // 后退：左轮快 → 给左轮减力
    // ==============================
    float vy = chassis->param.Vy_in;

    if(vy > 0.1f)  // 前进：左轮慢 → 加强左轮
    {
        chassis->param.V_out[0] *= 1.0129f;   // 左前
        chassis->param.V_out[2] *= 1.0129f;   // 左后
    }
    else if(vy < -0.1f)  // 后退：左轮快 → 削弱左轮
    {
        chassis->param.V_out[0] *= 0.95f;   // 左前
        chassis->param.V_out[2] *= 0.95f;   // 左后
    }

}

void Chassis_Stop(Chassis_Module *chassis)
{
    // 2. 将速度输入与输出清零
    chassis->param.Vx_in = 0.0f;
    chassis->param.Vy_in = 0.0f;
    chassis->param.Vw_in = 0.0f;
    chassis->param.V_out[0] = 0.0f;
    chassis->param.V_out[1] = 0.0f;
    chassis->param.V_out[2] = 0.0f;
    chassis->param.V_out[3] = 0.0f;

    // 3. PID 输出清零，防止残留
    chassis_motor1.pid_spd.Output = 0.0f;
    chassis_motor2.pid_spd.Output = 0.0f;
    chassis_motor3.pid_spd.Output = 0.0f;
    chassis_motor4.pid_spd.Output = 0.0f;

    guide_motor1.pid_spd.Output = 0.0f;
    guide_motor2.pid_spd.Output = 0.0f;
}


