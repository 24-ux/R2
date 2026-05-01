#include "kfs.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include <math.h>
#include "cmsis_os.h"
#include "Motion_Task.h"
#include "chassis.h"

Kfs_Module Kfs;

DJI_MotorModule kfs_above;  
DJI_MotorModule kfs_below;  

DM_MotorModule main_lift;
DM_MotorModule kfs_spin;
DM_MotorModule three_kfs;


Three_kfs_position three_kfs_position;
Kfs_spin_position kfs_spin_position;
Main_lift_position main_lift_position;

// �ϵ��ʼλ��
float main_lift_Initpos = 0.2f;
float kfs_spin_Initpos = 0.0f;
float three_kfs_Initpos = -4.055f;

float kfs_above_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float kfs_below_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};

// ��ʼ������ȡ�ϵ��ʼλ��
void kfs_three_kfs_spin_main_lift_pos_init(void)
{
	three_kfs.set_mit_data(&three_kfs, three_kfs_Initpos, 0.0f, 5.0f, 0.2f, 0.2f);
//	main_lift.set_mit_data(&main_lift, MAIN_LIFT_OFFSET1, 0.0f, 0.2, 0.15f, -5.0f);
 	kfs_spin.set_mit_data(&kfs_spin, kfs_spin_Initpos + KFS_SPIN_OFFSET1, 0.0f, 6.5f, 2.0f, 0.0f);

	three_kfs_position = three_kfs_p1;
	main_lift_position = main_lift_p1; /* ������ʼ����p1������ң�ؽ���p1~p4ѭ���� */
	kfs_spin_position  = kfs_spin_p1;
}

/**
  * @brief KFS�����߼�
  */
void manual_kfs_function(void)
{
	/* ң�ص�ģʽ�±���ԭ��Ϊ�����ز���ģʽ�²���ͣ���� */
	if (control_mode == remote_control)
	{
		Chassis.Chassis_Stop(&Chassis);
	}
	
	int16_t master_kfs_above_spd_cmd = 0;
	int16_t master_kfs_below_spd_cmd = 0;
	static Control_mode last_control_mode = remote_control;

	/* masterģʽ��KFSʹ�õ��ֽ�8λ������master_kfs_action_bits_0��
	 * bit0~1: ������ת 00/01/10��11Ԥ��
	 * bit2   : ǰ�۶��� 1/0
	 * bit3~5 : ����̧��״̬���� 0~6��7Ԥ��
	 * bit6~7 : ��������λ�� 00/01��10/11Ԥ��=ֹͣ
	 */
	if (control_mode == master_control)
	{
		uint16_t kfs_action_word = (uint16_t)master_kfs_action_bits_0 |
		                           ((uint16_t)master_kfs_action_bits_1 << 8);
		uint8_t action = (uint8_t)(kfs_action_word & 0xFFU);

		/* bit0~1: ������ת => three_kfs_position */
		switch (action & 0x03U)
		{
			case 0: three_kfs_position = three_kfs_p1; break; /* 00 */
			case 1: three_kfs_position = three_kfs_p2; break; /* 01 */
			case 2: three_kfs_position = three_kfs_p3; break; /* 10 */
			default: three_kfs_position = three_kfs_p1; break; /* 11Ԥ�� */
		}

		/* bit2: ǰ������ => kfs_spin_position */
		kfs_spin_position = ((action & (1U << 2)) != 0U) ? kfs_spin_p2 : kfs_spin_p1; /* 1/0 */

		/* bit3~5: ����̧����״̬ӳ��
		 * 001 -> ״̬1
		 * 010 -> ״̬2
		 * 011 -> ״̬3
		 * 100 -> ״̬4
		 * ����ֵԤ�������ֵ�ǰ״̬����
		 */
		{
			uint8_t lift_code = (uint8_t)((action >> 3) & 0x07U);
			switch (lift_code)
			{
				case 0x00U: main_lift_position = main_lift_p0; break; /* 000: ���� */
				case 0x01U: main_lift_position = main_lift_p1; break; /* 001 */
				case 0x02U: main_lift_position = main_lift_p2; break; /* 010 */
				case 0x03U: main_lift_position = main_lift_p3; break; /* 011 */
				case 0x04U: main_lift_position = main_lift_p4; break; /* 100 */
				default:
					/* reserved */
					break;
			}
		}

		/* ������������ƣ�
		 * �������ֽ�ƴ��16λ��ȡ4λ�����������ռ2λ���
		 * - above�������λ��bit7~6
		 * - below�������λ��bit9~8�����Եڶ��ֽڵ�2λ��
		 *
		 * above������룺
		 * 00 ����
		 * 01 ��� -> -100
		 * 10 �ջ� -> +100
		 * 11 Ԥ����������������
		 *
		 * below������루����ǰ��е���򣩣�
		 * 00 ����
		 * 01 �ջ� -> -100
		 * 10 ��� -> +100
		 * 11 Ԥ����������������
		 */
		{
			uint8_t above_cmd = (uint8_t)((kfs_action_word >> 6) & 0x03U);
			uint8_t below_cmd = (uint8_t)((kfs_action_word >> 8) & 0x03U);

			if (above_cmd == 0x01U)
			{
				master_kfs_above_spd_cmd = -2500;
			}
			else if (above_cmd == 0x02U)
			{
				master_kfs_above_spd_cmd = 2500;
			}
			else
			{
				master_kfs_above_spd_cmd = 0;
			}

			if (below_cmd == 0x01U)
			{
				master_kfs_below_spd_cmd = -2500; /* 01: �ջ� */
			}
			else if (below_cmd == 0x02U)
			{
				master_kfs_below_spd_cmd = 2500;  /* 10: ��� */
			}
			else
			{
				master_kfs_below_spd_cmd = 0;
			}
		}
	}

	/* ==================== ������ת ==================== */
	// ͨ��һ����������תKFS
	static uint16_t ch1_prev = 0;
	static int8_t three_kfs_pingpong_dir = 1; /* 1: p1->p3, -1: p3->p1 */
	
	if (control_mode == remote_control)
	{
		if (RCctrl.CH1 >=1500 && ch1_prev <=500)
		{
			if (three_kfs_position == three_kfs_p1) three_kfs_pingpong_dir = 1;
			else if (three_kfs_position == three_kfs_p3) three_kfs_pingpong_dir = -1;

			if (three_kfs_pingpong_dir > 0)
			{
				if (three_kfs_position == three_kfs_p1) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p3;
				else three_kfs_position = three_kfs_p2;
			}
			else
			{
				if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p1;
				else three_kfs_position = three_kfs_p2;
			}
		}
		if (RCctrl.CH1 <=500 && ch1_prev >=1500)
		{
			if (three_kfs_position == three_kfs_p1) three_kfs_pingpong_dir = 1;
			else if (three_kfs_position == three_kfs_p3) three_kfs_pingpong_dir = -1;

			if (three_kfs_pingpong_dir > 0)
			{
				if (three_kfs_position == three_kfs_p1) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p3;
				else three_kfs_position = three_kfs_p2;
			}
			else
			{
				if (three_kfs_position == three_kfs_p3) three_kfs_position = three_kfs_p2;
				else if (three_kfs_position == three_kfs_p2) three_kfs_position = three_kfs_p1;
				else three_kfs_position = three_kfs_p2;
			}
		}
		ch1_prev = RCctrl.CH1;
	}
	
	
	


	float tar_3k;
	const float kp_3k = 10.0f;
	const float kd_3k = 2.0f;
	const float tar_step_max_3k = 0.009f; 
	static float tar_3k_ramped = 0.0f;
	static uint8_t tar_3k_ramped_inited = 0U;
	
	switch(three_kfs_position)
	{
		case three_kfs_p1:
			tar_3k = THREE_KFS_OFFSET1;
			three_kfs.set_mit_data(&three_kfs, tar_3k_ramped, 0.0f, kp_3k, kd_3k, 0.0f);

		break;
		case three_kfs_p2:
			tar_3k = THREE_KFS_OFFSET2;
			three_kfs.set_mit_data(&three_kfs, tar_3k_ramped, 0.0f, kp_3k, kd_3k, 0.2f);

		break;
		case three_kfs_p3: 
			tar_3k = THREE_KFS_OFFSET3;
			three_kfs.set_mit_data(&three_kfs, tar_3k_ramped, 0.0f, kp_3k, kd_3k, 0.0f);

		break;
		default: tar_3k = three_kfs_Initpos;
	}

	if (tar_3k_ramped_inited == 0U)
	{
		tar_3k_ramped = three_kfs.position;
		tar_3k_ramped_inited = 1U;
	}
	{
		float delta = tar_3k - tar_3k_ramped;
		if (delta > tar_step_max_3k) delta = tar_step_max_3k;
		else if (delta < -tar_step_max_3k) delta = -tar_step_max_3k;
		tar_3k_ramped += delta;
	}
	// three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.0f, 0.0f, 0.0f);
	
	/* ==================== ����̧�� ==================== */
	/* --- [�����] ң��CH3 -> Ŀ�굵λ���� main_lift_position --- */
	static uint8_t main_lift_busy = 0U; /* ��������ȡ������æ��־ */
	
		/* ң�أ�CH3���ػ�������CH4�е����һ�£� */
		if (control_mode == remote_control)
		{
			static uint16_t ch3_prev = 0;
			static uint8_t ch3_cmd_lock = 0U; /* 1=���ᶯ��ִ���У������»������� */

			ch3_prev = RCctrl.CH3;
			/* ��ֵ���أ�����ҡ��ֵû��ȷ��192/1792ʱ������������ */
			{
				static uint8_t ch3_zone_prev = 1U; /* 0=LOW,1=MID,2=HIGH */
				uint8_t ch3_zone = 1U;
				if (RCctrl.CH3 >= 1500) ch3_zone = 2U;
				else if (RCctrl.CH3 <= 500) ch3_zone = 0U;

				/* ң�أ���p0~p4ѭ�����ϲ�=+1(ѭ��)���²�=-1(ѭ��) */
				if (ch3_zone == 2U && ch3_zone_prev != 2U && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position + 1) % 5);
				}
				if (ch3_zone == 0U && ch3_zone_prev != 0U && ch3_cmd_lock == 0U)
				{
					main_lift_position = (Main_lift_position)(((int)main_lift_position - 1 + 5) % 5);
				}
				ch3_zone_prev = ch3_zone;
			}
			ch3_cmd_lock = main_lift_busy;

		}
		/* --- [״̬��] ����̧��״̬�������ϴ�Ŀ��/λ�ù���/�˶���־�� --- */
		/* --- [ִ�в�������] ��λ�仯 -> �̶��ٶ� + �ֶμ�ʱ -> ��ʱֹͣ --- */
		{
			static Main_lift_position main_lift_cmd_prev = main_lift_p0;        /* ��һ����ִ�е�Ŀ�굵λ */
			static Main_lift_position main_lift_pos_est = main_lift_p0;         /* ��ǰλ�ù��Ƶ�λ����ʱ�����ƣ� */
			static Main_lift_position main_lift_target_active = main_lift_p0;   /* ��ǰ����ִ�е�Ŀ�굵λ */
			static Main_lift_position main_lift_target_pending = main_lift_p0;  /* �˶����յ�����Ŀ�꣨��ִ�У� */
			static uint8_t main_lift_pending_valid = 0U;                        /* ��ִ��Ŀ���Ƿ���Ч��1��Ч/0�� */
			static uint8_t lift_moving = 0U;                                    /* ��ʱ����״̬��1�˶���/0ֹͣ */
			static int8_t lift_dir = 0; /* +1������-1�½� */
			static uint32_t lift_move_end_tick = 0U;                            /* ���ζ�������ʱ�̣�tick�� */
			const float v_up = -2.5f;                                           /* �����̶��ٶ� */
			const float v_down = 2.5f;    
			//p0:000 p1:001 p2:010 p3:011 p4:100
			const uint32_t t_up_ms[4]   = {400U, 0U, 2080U, 1470U};
			const uint32_t t_down_ms[4] = {400U, 0U, 2080U, 1500U};

			if (control_mode == master_control || control_mode == remote_control)
			{
				/* --- [���Ȳ�] Ŀ���ٲã��˶��л���pending������ʱ��active --- */
				/* ͳһ������������ִ���в�������Ŀ�꣬�Ȼ��棬�ȵ�ǰ�����������л� */
				if (lift_moving != 0U)
				{
					if (main_lift_position != main_lift_target_active)
					{
						main_lift_target_pending = main_lift_position;
						main_lift_pending_valid = 1U;
					}
				}
				else
				{
					if (main_lift_pending_valid != 0U)
					{
						main_lift_target_active = main_lift_target_pending;
						main_lift_pending_valid = 0U;
					}
					else
					{
						main_lift_target_active = main_lift_position;
					}
				}

				/* --- [��ʱ��] ��Ŀ�괥��������ʱ���뷽������һ�ζ��� --- */
				if (main_lift_target_active != main_lift_cmd_prev)
				{
					uint32_t duration = 0U;

					if ((int32_t)main_lift_target_active > (int32_t)main_lift_pos_est)
					{
						int32_t lvl = (int32_t)main_lift_pos_est;
						while (lvl < (int32_t)main_lift_target_active)
						{
							if (lvl >= 0 && lvl <= 3) duration += t_up_ms[lvl];
							lvl++;
						}
						lift_dir = +1;
						if (duration > 0U)
						{
							lift_moving = 1U;
							lift_move_end_tick = osKernelGetTickCount() + duration;
						}
						else
						{
							lift_moving = 0U;
						}
					}
					else if ((int32_t)main_lift_target_active < (int32_t)main_lift_pos_est)
					{
						int32_t lvl = (int32_t)main_lift_pos_est;
						while (lvl > (int32_t)main_lift_target_active)
						{
							if (lvl >= 1 && lvl <= 4) duration += t_down_ms[lvl - 1];
							lvl--;
						}
						lift_dir = -1;
						if (duration > 0U)
						{
							lift_moving = 1U;
							lift_move_end_tick = osKernelGetTickCount() + duration;
						}
						else
						{
							lift_moving = 0U;
						}
					}
					else
					{
						lift_moving = 0U;
						lift_dir = 0;
					}

					main_lift_cmd_prev = main_lift_target_active;
				}

				/* --- [ִ�в�] �˶��з��ٶȣ���ʱ��ͣ��������λ�ù��� --- */
				if (lift_moving != 0U)
				{
					/* �����з��򶵵ף���ֹlift_dirż��Ϊ0���²����ٶȷ�֧ */
					if (lift_dir == 0)
					{
						if ((int32_t)main_lift_cmd_prev > (int32_t)main_lift_pos_est) lift_dir = +1;
						else if ((int32_t)main_lift_cmd_prev < (int32_t)main_lift_pos_est) lift_dir = -1;
					}
					if ((int32_t)(lift_move_end_tick - osKernelGetTickCount()) <= 0)
					{
						lift_moving = 0U;
						main_lift_pos_est = main_lift_cmd_prev;
						lift_dir = 0;
						main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.3f, -1.0f);
					}
					else
					{
						if (lift_dir > 0) main_lift.set_mit_data(&main_lift, 0.0f, v_up, 0.0f, 0.3f, -1.0f);
						else if (lift_dir < 0) main_lift.set_mit_data(&main_lift, 0.0f, v_down, 0.0f, 0.3f, -1.0f);
						else main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.3f, -1.0f);
					}
				}
				else
				{
					main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.3f, -1.0f);
				}
				main_lift_busy = lift_moving;
			}
			else
			{
				main_lift_busy = 0U;
				main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			}
		}

	/* ==================== ǰ����ת ==================== */


	static uint16_t ch4_prev = 0;

		if (control_mode == remote_control)
		{
			if (RCctrl.CH4 >=1500 && ch4_prev <=500)
			{
				kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position + 1) % 2);
			}
			if (RCctrl.CH4<=500 && ch4_prev >=1500)
			{
				kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position - 1+2) % 2);
			}
			ch4_prev = RCctrl.CH4;
		}

float tar_spin;
	switch(kfs_spin_position)
	{
		case kfs_spin_p1:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET1;
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 11.0f, 2.6f, -4.0f);
		break;
		case kfs_spin_p2:
			tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET2;
			// kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 6.8f, 2.2f, 0.0f);
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.3f, 0.4f, 0.0f);
		break;
	}


	
	
	/* ==================== �������� ==================== */
	// ͨ������������
	

		if (control_mode == master_control)
		{
			kfs_above.PID_Calculate(&kfs_above, master_kfs_above_spd_cmd);
			kfs_below.PID_Calculate(&kfs_below, master_kfs_below_spd_cmd);
		}
		// CH5�л����Ƶ��
		else if (control_mode == remote_control)
		{
			/* ������ģʽ�л�ң��ʱ��ͬ����һ�����룬����CH5�����󴥷� */
			if (last_control_mode != remote_control)
			{
				ch5_prev = RCctrl.CH5;
			}

			if (RCctrl.CH5 == CH5_LOW && ch5_prev != CH5_LOW)
			{
				kfs_motor_select = !kfs_motor_select;
			}
			ch5_prev = RCctrl.CH5;

			if(kfs_motor_select==0)
			{
				kfs_above.PID_Calculate(&kfs_above,(992-RCctrl.CH2)*8);
				kfs_below.PID_Calculate(&kfs_below,0);
			}
			else
			{
				kfs_above.PID_Calculate(&kfs_above,0);
				kfs_below.PID_Calculate(&kfs_below,(RCctrl.CH2-992)*8);
			}
		}
		else
		{
			/* ����ģʽ���������������� */
			kfs_above.PID_Calculate(&kfs_above, 0);
			kfs_below.PID_Calculate(&kfs_below, 0);
		}

		last_control_mode = control_mode;


}
