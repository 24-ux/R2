#include "chassis_yaw_correction.h"
#include <math.h>

/* ========================= 可调参数区 ========================= */
/* 单环角度 PI(D)（角度误差 -> 纠偏输出，叠加到 Vx_in） */
#define CHASSIS_YAW_KP                   (30.0f)
#define CHASSIS_YAW_KI                   (0.8f)
#define CHASSIS_YAW_KD                   (3.0f)     /* D 用 -gyro_z(rad/s) 实现阻尼 */
#define CHASSIS_YAW_I_LIMIT              (100.0f)

/* 输出与判据 */
#define CHASSIS_YAW_MOVE_OUT_LIMIT       (15.0f)    /* 移动中纠偏上限：过大会干扰左右平移 */
#define CHASSIS_YAW_STOP_OUT_LIMIT       (50.0f)    /* 松杆/停车锁角上限：用于压扫尾 */
#define CHASSIS_YAW_ROT_DEADZONE         (2.0f)     /* |Vx_in| 超过则认为手在主动转向 */
#define CHASSIS_YAW_MOVE_DEADZONE        (0.2f)     /* |Vy_in|/|Vw_in| 超过则认为在移动 */
#define CHASSIS_YAW_CORR_SIGN            (-1.0f)    /* 越控越偏就翻转正负号 */

/* gyro_z 预处理 */
#define CHASSIS_GYRO_Z_LPF_CUTOFF_HZ     (40.0f)
#define CHASSIS_GYRO_Z_DEADBAND_RADPS    (0.04f)

/* 松杆/停车锁角（抑制滑地扫尾）：角度P + 角速度D + 软启动 + 输出斜率限制 */
#define CHASSIS_YAW_STOP_HOLD_MS         (1500U)    /* 松杆后最多锁角多久 */
#define CHASSIS_YAW_STOP_RAMP_MS         (220U)     /* 略增软启动：在原有效参数基础上减小松杆抖动 */
#define CHASSIS_YAW_STOP_KP              (40.0f)
#define CHASSIS_YAW_STOP_KD              (10.0f)
#define CHASSIS_YAW_STOP_MIN_OUT         (20.0f)    /* 最小反向输出（像手动轻打一把旋转） */
#define CHASSIS_YAW_STOP_ERR_TH          (0.01f)    /* |yaw_err| 触发最小输出阈值(rad) */
#define CHASSIS_YAW_STOP_GYRO_TH         (0.05f)    /* |gyro_z| 触发最小输出阈值(rad/s) */
#define CHASSIS_YAW_STOP_SLEW_PER_S      (350.0f)   /* 中等斜率限制：比原版更稳、比当前更有力 */
/* 高速自适应：松杆瞬间速度越大，锁角越强、介入越快；低速保持柔和 */
#define CHASSIS_YAW_STOP_SPEED_REF       (100.0f)   /* 速度归一化参考：约等于常用平移指令幅值 */
#define CHASSIS_YAW_STOP_GAIN_BOOST      (0.0f)     /* 回到基础参数：关闭速度自适应增益 */
#define CHASSIS_YAW_STOP_MINOUT_BOOST    (0.0f)     /* 回到基础参数：关闭最小输出自适应 */
#define CHASSIS_YAW_STOP_RAMP_REDUCE_MS  (0U)       /* 回到基础参数：关闭高速缩短软启动 */
#define CHASSIS_YAW_STOP_RAMP_MIN_MS     (80U)      /* 软启动下限，防止突兀 */
#define CHASSIS_YAW_STOP_SLEW_BOOST      (0.0f)     /* 回到基础参数：关闭斜率自适应 */

/* 完全停车后的轻阻尼（避免残余抖动） */
#define CHASSIS_YAW_STOP_DAMP_KD         (12.0f)

/* ========================= 内部状态区 ========================= */
static float s_yaw_est = 0.0f;        /* 航向估计值 yaw(rad)：由 gyro_z(rad/s) 积分得到并 wrap 到 [-pi, pi] */
static float s_yaw_target = 0.0f;     /* 航向保持目标 yaw(rad)：进入保持时锁定为当前 s_yaw_est */
static float s_yaw_i = 0.0f;          /* 角度 PID 的 I 项累加器（输出量纲），受 g_chassis_yaw_i_limit 限幅 */
static uint8_t s_hold_active = 0U;    /* 航向保持状态标志：0=未保持，1=保持中（用于首次进入时锁目标） */
static uint32_t s_last_tick = 0U;     /* 上次调用的系统 tick(ms)：用于计算 dt */
static float s_gyro_z_filt = 0.0f;    /* gyro_z 一阶低通后的滤波状态/输出(rad/s) */
static uint32_t s_last_move_tick = 0U;/* 最近一次检测到平移指令的 tick(ms)：用于松杆收尾窗口判定 */
static uint8_t s_last_has_move_cmd = 0U; /* 上一次是否有平移指令：用于检测松杆边沿 */
static uint32_t s_release_tick = 0U;     /* 松杆时间戳(ms)：用于锁角窗口与软启动 */
static float s_yaw_out_prev = 0.0f;      /* 上一次输出：用于斜率限制平滑 */
static float s_release_speed_n = 0.0f;    /* 松杆瞬间平移速度归一化值[0,1]：用于高速自适应增益 */

static float wrap_pi(float a)
{
    const float pi = 3.1415926f;
    const float two_pi = 6.2831853f;
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

static float sign_from_err_or_gyro(float yaw_err, float gyro_z)
{
    if (yaw_err > 0.0f) return 1.0f;
    if (yaw_err < 0.0f) return -1.0f;
    if (gyro_z > 0.0f) return 1.0f;
    if (gyro_z < 0.0f) return -1.0f;
    return 0.0f;
}

void chassis_yaw_corr_reset(void)
{
    s_yaw_est = 0.0f;
    s_yaw_target = 0.0f;
    s_yaw_i = 0.0f;
    s_hold_active = 0U;
    s_last_tick = 0U;
    s_gyro_z_filt = 0.0f;
    s_last_move_tick = 0U;
    s_last_has_move_cmd = 0U;
    s_release_tick = 0U;
    s_yaw_out_prev = 0.0f;
    s_release_speed_n = 0.0f;
}

void chassis_yaw_corr_apply(Chassis_Module *chassis, float gyro_z_radps)
{
    uint32_t now_tick = HAL_GetTick();
    float dt = 0.003f;
    float gyro_z;
    float vx_in_raw;
    uint8_t has_rot_cmd;
    uint8_t has_move_cmd;
    float yaw_err = 0.0f;
    float yaw_out;

    if (chassis == NULL) return;

    vx_in_raw = chassis->param.Vx_in; /* 只代表“用户/上层给的旋转指令”，不包含本函数叠加的纠偏 */

    if (s_last_tick != 0U && now_tick >= s_last_tick)
    {
        dt = (float)(now_tick - s_last_tick) * 0.001f;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.02f)  dt = 0.02f;
    }
    s_last_tick = now_tick;

    /* gyro_z 预处理：一阶低通 + 死区 */
    gyro_z = gyro_z_radps;
    {
        const float fc = CHASSIS_GYRO_Z_LPF_CUTOFF_HZ;
        const float tau = 1.0f / (6.2831853f * fc);
        const float alpha = dt / (tau + dt);
        s_gyro_z_filt = s_gyro_z_filt + alpha * (gyro_z - s_gyro_z_filt);
        gyro_z = s_gyro_z_filt;
        if (fabsf(gyro_z) < CHASSIS_GYRO_Z_DEADBAND_RADPS) gyro_z = 0.0f;
    }

    /* 航向估计 */
    s_yaw_est = wrap_pi(s_yaw_est + gyro_z * dt);

    /* 输入判据：Vx_in 为旋转通道；Vy_in/Vw_in 为平移通道 */
    /* 注意：这里必须用“原始 Vx_in”，否则纠偏叠加过大时会被误判成手动旋转，导致保持被自己打断 */
    has_rot_cmd = (fabsf(vx_in_raw) > CHASSIS_YAW_ROT_DEADZONE) ? 1U : 0U;
    has_move_cmd = (fabsf(chassis->param.Vy_in) > CHASSIS_YAW_MOVE_DEADZONE ||
                    fabsf(chassis->param.Vw_in) > CHASSIS_YAW_MOVE_DEADZONE) ? 1U : 0U;

    if (has_move_cmd != 0U) s_last_move_tick = now_tick;

    /* 松杆边沿：在“有平移 -> 无平移”的瞬间锁定目标角（你要的“松杆前角度”） */
    if (s_last_has_move_cmd != 0U && has_move_cmd == 0U)
    {
        float release_speed = sqrtf(chassis->param.Vy_in * chassis->param.Vy_in +
                                    chassis->param.Vw_in * chassis->param.Vw_in);
        s_release_tick = now_tick;
        s_hold_active = 1U;
        s_yaw_target = s_yaw_est;
        s_yaw_i = 0.0f;
        s_yaw_out_prev = 0.0f;
        s_release_speed_n = clampf_local(release_speed / CHASSIS_YAW_STOP_SPEED_REF, 0.0f, 1.0f);
    }
    s_last_has_move_cmd = has_move_cmd;

    /* 人工旋转：退出保持 */
    if (has_rot_cmd != 0U)
    {
        s_hold_active = 0U;
        s_yaw_i = 0.0f;
        s_yaw_target = s_yaw_est;
        return;
    }

    /* 松杆/停车：优先“锁住松杆前的目标角”，用反向旋转把 yaw 拉回去抑制扫尾 */
    if (has_move_cmd == 0U)
    {
        uint8_t stop_hold_active = 0U;

        if (s_last_move_tick != 0U && now_tick >= s_last_move_tick)
        {
            if ((now_tick - s_last_move_tick) <= CHASSIS_YAW_STOP_HOLD_MS) stop_hold_active = 1U;
        }

        if (stop_hold_active != 0U)
        {
            const float gain_scale = 1.0f + (s_release_speed_n * CHASSIS_YAW_STOP_GAIN_BOOST);
            const float kp_eff = CHASSIS_YAW_STOP_KP * gain_scale;
            const float kd_eff = CHASSIS_YAW_STOP_KD * gain_scale;
            const float min_out_eff = CHASSIS_YAW_STOP_MIN_OUT *
                                      (1.0f + s_release_speed_n * CHASSIS_YAW_STOP_MINOUT_BOOST);
            const float slew_eff = CHASSIS_YAW_STOP_SLEW_PER_S *
                                   (1.0f + s_release_speed_n * CHASSIS_YAW_STOP_SLEW_BOOST);

            yaw_err = wrap_pi(s_yaw_target - s_yaw_est);

            /* 简单有效：角度误差决定方向 + 角速度反向阻尼 */
            yaw_out = (kp_eff * yaw_err) - (kd_eff * gyro_z);

            /* 最小反向输出：避免输出太小（像你手动轻打一把旋转） */
            if ((fabsf(yaw_err) > CHASSIS_YAW_STOP_ERR_TH) ||
                (fabsf(gyro_z) > CHASSIS_YAW_STOP_GYRO_TH))
            {
                const float min_out = fabsf(min_out_eff);
                const float sgn = sign_from_err_or_gyro(yaw_err, gyro_z);
                if (sgn != 0.0f && fabsf(yaw_out) < min_out)
                {
                    yaw_out = sgn * min_out;
                }
            }

            /* 软启动：松杆瞬间逐步放大输出，抑制抖一下 */
            if (s_release_tick != 0U && now_tick >= s_release_tick && CHASSIS_YAW_STOP_RAMP_MS > 0U)
            {
                uint32_t t = now_tick - s_release_tick;
                uint32_t ramp_eff_ms = CHASSIS_YAW_STOP_RAMP_MS;
                uint32_t reduce_ms = (uint32_t)(s_release_speed_n * (float)CHASSIS_YAW_STOP_RAMP_REDUCE_MS);
                if (reduce_ms < ramp_eff_ms) ramp_eff_ms -= reduce_ms;
                if (ramp_eff_ms < CHASSIS_YAW_STOP_RAMP_MIN_MS) ramp_eff_ms = CHASSIS_YAW_STOP_RAMP_MIN_MS;
                if (t < ramp_eff_ms)
                {
                    float k = (float)t / (float)ramp_eff_ms;
                    if (k < 0.0f) k = 0.0f;
                    if (k > 1.0f) k = 1.0f;
                    yaw_out *= k;
                }
            }

            /* 输出斜率限制：限制 yaw_out 的变化速度，进一步消除“抖一下” */
            {
                const float max_step = slew_eff * dt;
                const float lo = s_yaw_out_prev - max_step;
                const float hi = s_yaw_out_prev + max_step;
                yaw_out = clampf_local(yaw_out, lo, hi);
                s_yaw_out_prev = yaw_out;
            }

            yaw_out = clampf_local(yaw_out, -CHASSIS_YAW_STOP_OUT_LIMIT, CHASSIS_YAW_STOP_OUT_LIMIT);
            chassis->param.Vx_in += (CHASSIS_YAW_CORR_SIGN * yaw_out);
            return;
        }

        /* 锁角窗口结束后：只做轻微角速度阻尼收敛 */
        yaw_out = -CHASSIS_YAW_STOP_DAMP_KD * gyro_z;
        yaw_out = clampf_local(yaw_out, -CHASSIS_YAW_STOP_OUT_LIMIT, CHASSIS_YAW_STOP_OUT_LIMIT);
        chassis->param.Vx_in += (CHASSIS_YAW_CORR_SIGN * yaw_out);
        return;
    }

    /* 首次进入保持：锁定目标航向 */
    if (s_hold_active == 0U)
    {
        s_hold_active = 1U;
        s_yaw_target = s_yaw_est;
        s_yaw_i = 0.0f;
    }

    /* 单环角度 PID：D 项用 -gyro_z 做阻尼 */
    yaw_err = wrap_pi(s_yaw_target - s_yaw_est);
    s_yaw_i = clampf_local(s_yaw_i + CHASSIS_YAW_KI * yaw_err * dt,
                           -CHASSIS_YAW_I_LIMIT, CHASSIS_YAW_I_LIMIT);
    yaw_out = (CHASSIS_YAW_KP * yaw_err) + s_yaw_i - (CHASSIS_YAW_KD * gyro_z);
    yaw_out = clampf_local(yaw_out, -CHASSIS_YAW_MOVE_OUT_LIMIT, CHASSIS_YAW_MOVE_OUT_LIMIT);

    chassis->param.Vx_in += (CHASSIS_YAW_CORR_SIGN * yaw_out);
}

