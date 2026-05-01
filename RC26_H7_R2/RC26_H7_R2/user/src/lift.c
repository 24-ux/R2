#include "lift.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>
#include "Motion_Task.h"
#include "dm_motor.h"
#include "master_control.h"
#include "chassis.h"


//̧��
Lift_Module Lift;
DM_MotorModule R2_lift_motor_left;//����
DM_MotorModule R2_lift_motor_right;//���ң�

//����
DJI_MotorModule flexible_motor1;//����
DJI_MotorModule flexible_motor2;//���ң�

//̧��״̬��״̬
static uint8_t lift_has_stopped = 0;   // 1=�Ѵ���λͣ��
static uint8_t lift_running = 0;
int    lift_stop_mode  = 0;     // ��¼������ͣ�����½�ͣ�����ڸ�ɲ������
uint8_t lift_fall_fast = 0;
uint8_t lift_rise_fast = 0;


//����״̬
FlexibleMotorCmd flex_cmd = FLEX_CMD_NONE;               // ��������ɵı���������
FlexibleMotorState4 flex_state4 = FLEX_ST_RETRACTED;     // ��״̬״̬����ǰ״̬
uint16_t flex_input_prev = CH2_MID;                      // ��һ��������ֵ(���ڱ��ش���)
uint8_t flex_seen_move = 0;                              // ���ζ����Ƿ��Ѿ���⵽��ȷʵת����
uint8_t flex_stop_cnt = 0;                               // ������������(�ж�����ͷ)
float flexible_motor_PID_input;                         //��������PID����



float flexible_motor1_pid_param[PID_PARAMETER_NUM] = {5.0f,0.4f,0.2f,1,500.0f,10000.0f};
float flexible_motor2_pid_param[PID_PARAMETER_NUM] = {5.0f,0.4f,0.2f,1,500.0f,10000.0f};

/* ����������CH4���ֵ�����ߵ��α��� */
volatile float lift_rise_fast_left_v  = 3.0f;
volatile float lift_rise_fast_kp = 0.15f;
volatile float lift_rise_fast_kd = 0.15f;
volatile float lift_rise_fast_left_t  = 3.6f;

volatile float lift_rise_fast_right_v  = -3.4f;
volatile float lift_rise_fast_right_t  = -3.9f;

void lift_init()
{
    // ��ʼ��Ĭ��״̬���½� fall
    r2_lift_mode = fall;



    // ��λ��λ���״̬
    lift_has_stopped = 0;
    lift_running    = 0;
    lift_stop_mode  = 0;
    lift_fall_fast  = 0;
    lift_rise_fast  = 0;


	// flexible_motor ��״̬���ϵ��ֵ
	flex_cmd = FLEX_CMD_NONE;
	flex_state4 = FLEX_ST_RETRACTED;
	flex_input_prev = CH2_MID;
	flex_seen_move = 0;
	flex_stop_cnt = 0;
	flexible_motor_PID_input = 0.0f;

}

void manual_lift_function(void)
{
	/* ң�ص�ģʽ�±���ԭ��Ϊ�����ز���ģʽ�²�Ҫ��ͣ���� */
	if (control_mode == remote_control)
	{
		Chassis.Chassis_Stop(&Chassis);
	}
	
	static MasterLevelGate master_lift_flex_gate = {0U, 0U};
	static MasterLevelGate master_lift_updown_gate = {0U, 0U};

	if (control_mode == master_control)
	{
		uint8_t flex_level = ((master_lift_action_bits & MASTER_LIFT_FLEX_BIT) != 0U) ? 1U : 0U;
		uint8_t updown_level = ((master_lift_action_bits & MASTER_LIFT_UPDOWN_BIT) != 0U) ? 1U : 0U;
		uint8_t fall_fast_level = ((master_lift_action_bits & MASTER_LIFT_FALL_FAST_BIT) != 0U) ? 1U : 0U;
		uint8_t rise_fast_level = ((master_lift_action_bits & MASTER_LIFT_RISE_FAST_BIT) != 0U) ? 1U : 0U;

		/* masterģʽ��
		 * bit0 ̧������1������0�½�������ƽ�仯һ�δ�����
		 * bit1 �����½�����ƽΪ1�ҵ�ǰΪ�½�ָ��ʱ�� lift_fall_fast����ң�� CH4 ��סһ�£�
		 * bit2 ������������ƽΪ1�ҵ�ǰΪ����ָ��ʱ�� lift_rise_fast����ң�� CH4 ��סһ�£�
		 * bit3 ��������1�����0�ջ�
		 */
		/* ̧������ͬ��������ƽ�仯һ�δ��������� */
		if (master_level_gate_on_change(&master_lift_updown_gate, updown_level) != 0U)
		{
			r2_lift_mode = updown_level ? raise : fall;
		}

		/* �½� + ����λ����ң�ط�֧��ÿ����д lift_fall_fast=1 ��Ч */
		if (r2_lift_mode == fall && fall_fast_level != 0U)
		{
			lift_fall_fast = 1U;
		}
		if (r2_lift_mode == raise && rise_fast_level != 0U)
		{
			lift_rise_fast = 1U;
		}

		/* ��ʹ������ͬһ��ƽ��Ҳֻ�ڵ�ƽ�仯ʱ����һ���������� */
		if (master_level_gate_on_change(&master_lift_flex_gate, flex_level) != 0U)
		{
			/* masterģʽ��ֱ�Ӱ�bit�����·��������CH2ֵӳ����������� */
			flex_cmd = flex_level ? FLEX_CMD_EXTEND : FLEX_CMD_RETRACT;
		}
		else
		{
			/* ���±���ʱ���ظ����� */
			flex_cmd = FLEX_CMD_NONE;
		}
	}

	 
	else if(control_mode == remote_control)
	{
		/* ң��ģʽ�½��ſ�״̬�롰��ʵ����״̬�����룬�����´��л�master�󴥷�
		 * ע�⣺��λ��ҡ��/ָ����ܻ���λ��������״̬�����Զ�������˲�����ͨ��ֵ����������
		 */
		{
			uint8_t flex_real_level = 0U;
			if (flex_state4 == FLEX_ST_EXTENDED) flex_real_level = 1U;
			else if (flex_state4 == FLEX_ST_RETRACTED) flex_real_level = 0U;
			/* �����˶���״̬����Ĭ�ϼ��� */
			master_level_gate_init(&master_lift_flex_gate, flex_real_level);
		}
		{
			uint8_t updown_real_level;
			/* ���Ȱ�����ʵ��λ״̬�����룺
			 * - �ѵ�λ�� stop_mode=raise: ��Ϊ��ǰ��������
			 * - �ѵ�λ�� stop_mode=fall : ��Ϊ��ǰ���½���
			 * δ��λʱ�ٻ��˵���ǰָ��״̬ r2_lift_mode
			 */
			if (lift_has_stopped != 0U)
			{
				updown_real_level = (lift_stop_mode == raise) ? 1U : 0U;
			}
			else
			{
				updown_real_level = (r2_lift_mode == raise) ? 1U : 0U;
			}
			master_level_gate_init(&master_lift_updown_gate, updown_real_level);
		}

		if(RCctrl.CH3>=1500)
		r2_lift_mode = raise;  // ����
		else if(RCctrl.CH3<=500)
		r2_lift_mode = fall;   // ����
		else if(RCctrl.CH4<=500)
		{
			r2_lift_mode = fall;   // ����
			lift_fall_fast = 1;
		}
		else if(RCctrl.CH4>=1500)
		{
			r2_lift_mode = raise;
			lift_rise_fast = 1;
		}

		//����flexible_motor����
		flexible_motor_update_command(RCctrl.CH2);
	}


	flexible_motor_state_machine_step();

	flexible_motor1.PID_Calculate(&flexible_motor1,flexible_motor_PID_input);
	flexible_motor2.PID_Calculate(&flexible_motor2,-flexible_motor_PID_input);
				
				
	// ==================== ����������������޸� ====================
	static int last_r2_lift_mode = -1;

	// ģʽ�л� �� ��λ����״̬
	if(r2_lift_mode != last_r2_lift_mode)
	{
		last_r2_lift_mode = r2_lift_mode;
		lift_has_stopped = 0;
		lift_running = 0;
		lift_fall_fast = 0;
		lift_rise_fast = 0;
	}
	// �Ѿ�����/����ֹͣ �� ���ɲ�����أ�������
	  if(lift_has_stopped)
	{
		
		if(lift_stop_mode == fall)
		{
			// ������������΢С�������ض�ס���»�
				R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f,  -0.7f);
				R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f, 1.0f);
		}
		else if(lift_stop_mode == raise)
		{
				R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, 0, 0, 0.5f, 2.1f);
				R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 0, 0, 0.5f,  -3.0f);
		}
	}

	// ��������
	else if(r2_lift_mode == fall)
	{
		if (lift_fall_fast == 0)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -1.0f, 0, 0.30f, -1.3f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 1.0f, 0, 0.30f,  1.5f);
		}
		else if (lift_fall_fast != 0)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, -2.0f, 0, 0.30f, -3.1f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, 2.0f, 0, 0.30f, 3.1f);
		}


		if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || fabsf(R2_lift_motor_right.speed_w) > 1.5f)
		{
				lift_running = 1;

		}

		// ����ֹͣ
		if(lift_running && 
			 fabsf(R2_lift_motor_left.speed_w) < 0.5f && fabsf(R2_lift_motor_right.speed_w) < 0.5f)
		{
				lift_has_stopped = 1;
				lift_stop_mode = fall;  // ��¼ֹͣģʽ
				lift_fall_fast = 0;
		}
	}
	else if(r2_lift_mode == raise)
	{
		if (lift_rise_fast == 0U)
		{
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0,  2.8f, 0, 0.11f,  3.8f);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, -3.3f, 0, 0.11f, -4.1f);
		}
		else
		{  
			R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0, lift_rise_fast_left_v, lift_rise_fast_kp, lift_rise_fast_kd, lift_rise_fast_left_t);
			R2_lift_motor_right.set_mit_data(&R2_lift_motor_right,0, lift_rise_fast_right_v, lift_rise_fast_kp, lift_rise_fast_kd, lift_rise_fast_right_t);
		}

		if(fabsf(R2_lift_motor_left.speed_w) > 1.5f || fabsf(R2_lift_motor_right.speed_w) > 1.5f)
		{
				lift_running = 1;

		}

		// ����ֹͣ
		if(lift_running && 
			 fabsf(R2_lift_motor_left.speed_w) < 0.5f && fabsf(R2_lift_motor_right.speed_w) < 0.5f)
		{
				lift_has_stopped = 1;
				lift_stop_mode = raise; // ��¼ֹͣģʽ
				lift_rise_fast = 0;
		}
	}
}



void flexible_motor_update_command(uint16_t ch_value)
{
	// ÿ������������ֻ�м�⵽���زŲ���һ������
	flex_cmd = FLEX_CMD_NONE;

	// ��λ���أ����������������ָ�λ���ظ�������
	if (ch_value >=1500 && flex_input_prev <=1500)
	{
		flex_cmd = FLEX_CMD_RETRACT;
	}
	// ��λ���أ������ջء�������ֵ�λ���ظ�������
	else if (ch_value <=500 && flex_input_prev >=500)
	{
		flex_cmd = FLEX_CMD_EXTEND;
	}

	// ��¼��ǰ����������һ���ڵı��رȽ�
	flex_input_prev = ch_value;
}

void flexible_motor_state_machine_step(void)
{
	// ��ȡ˫�����ǰ�ٶȣ�ȡ����ֵ���ڡ��Ƿ��ڶ�/�Ƿ�ֹͣ���ж���
	int rpm1 = abs((int)flexible_motor1.speed_rpm);
	int rpm2 = abs((int)flexible_motor2.speed_rpm);

	// ����� -> ״̬�㣺�յ�һ������ͽ����Ӧ���˶��С�״̬�������㱾�ζ����ж�
	if (flex_cmd == FLEX_CMD_EXTEND)
	{
		flex_state4 = FLEX_ST_EXTENDING;
		flex_seen_move = 0;
		flex_stop_cnt = 0;
	}
	else if (flex_cmd == FLEX_CMD_RETRACT)
	{
		flex_state4 = FLEX_ST_RETRACTING;
		flex_seen_move = 0;
		flex_stop_cnt = 0;
	}

	switch (flex_state4)
	{
	case FLEX_ST_EXTENDING:
		// ����У��������������
		flexible_motor_PID_input = FLEX_CMD_EXTEND_PWM;
		if (rpm1 > FLEX_RUN_THR_RPM || rpm2 > FLEX_RUN_THR_RPM)
			flex_seen_move = 1;

		// �ȡ����������С��������١�����ͷ��������л�ʱ����
		if (flex_seen_move && rpm1 < FLEX_STOP_THR_RPM && rpm2 < FLEX_STOP_THR_RPM)
		{
			if (++flex_stop_cnt >= FLEX_STOP_CNT_MAX)
			{
				// �쵽��ͷ������̬��������
				flexible_motor_PID_input = 0.0f;
				flex_state4 = FLEX_ST_EXTENDED;
				flex_stop_cnt = 0;
			}
		}
		else
		{
			flex_stop_cnt = 0;
		}
		break;

	case FLEX_ST_RETRACTING:
		// �ջ��У��������ջ�����
		flexible_motor_PID_input = FLEX_CMD_RETRACT_PWM;
		if (rpm1 > FLEX_RUN_THR_RPM || rpm2 > FLEX_RUN_THR_RPM)
			flex_seen_move = 1;

		// �ȡ����������С��������١�����ͷ��������л�ʱ����
		if (flex_seen_move && rpm1 < FLEX_STOP_THR_RPM && rpm2 < FLEX_STOP_THR_RPM)
		{
			if (++flex_stop_cnt >= FLEX_STOP_CNT_MAX)
			{
				// �յ���ͷ������̬��������
				flexible_motor_PID_input = 0.0f;
				flex_state4 = FLEX_ST_RETRACTED;
				flex_stop_cnt = 0;
			}
		}
		else
		{
			flex_stop_cnt = 0;
		}
		break;

	case FLEX_ST_EXTENDED:
		// �쵽��ͷ��̬��������������
		flexible_motor_PID_input = 0.0f;
		break;

	case FLEX_ST_RETRACTED:
		// �յ���ͷ��̬��̧�������׶��Ա����ջ�����
		if (r2_lift_mode == raise)
			flexible_motor_PID_input = 500.0f;
		else
			flexible_motor_PID_input = 0.0f;
		break;

	default:
		flexible_motor_PID_input = 0.0f;
		break;
	}
}