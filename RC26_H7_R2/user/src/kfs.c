#include "kfs.h"
#include "remote_control.h"
#include "main.h"
#include "tim.h"
#include <math.h>
#include "cmsis_os.h"
#include "Motion_Task.h"

Kfs_Module Kfs;

DJI_MotorModule kfs_above;  
DJI_MotorModule kfs_below;  

DM_MotorModule main_lift;
DM_MotorModule kfs_spin;
DM_MotorModule three_kfs;

Three_kfs_position three_kfs_position;
Kfs_spin_position kfs_spin_position;
Main_lift_position main_lift_position;

//上电初始位置
float main_lift_Initpos = 0.2f;
float kfs_spin_Initpos = 0.0f;
float three_kfs_Initpos = 1.6f;

float kfs_above_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};
float kfs_below_pid_param[PID_PARAMETER_NUM] = {5.0f,0.1f,0.2f,1,500.0f,10000.0f};


//初始化：读取上电初始位置
void kfs_three_kfs_spin_main_lift_pos_init(void)
{
	three_kfs.set_mit_data(&three_kfs, three_kfs_Initpos, 0.0f, 0.5f, 0.2f, 0.2f);
	main_lift.set_mit_data(&main_lift, MAIN_LIFT_OFFSET1, 0.0f, 0.2, 0.15f, -5.0f);
	// kfs_spin.set_mit_data(&kfs_spin, kfs_spin_Initpos + KFS_SPIN_OFFSET1, 0.0f, 6.5f, 2.0f, 0.0f);

	three_kfs_position = three_kfs_p1;
	main_lift_position = main_lift_p1;
	kfs_spin_position  = kfs_spin_p1;
}

static float three_kfs_calc_feedforward_torque(float current_position, float target_position)
{
	float three_kfs_position_error = current_position - target_position;
	return 0.5123f * sinf(2.417f * three_kfs_position_error);
}


/**
  * @brief kfs函数
  */
void manual_kfs_function(void)
{
	int16_t master_kfs_above_spd_cmd = 0;
	int16_t master_kfs_below_spd_cmd = 0;
	static Control_mode last_control_mode = remote_control;

	/* master模式：KFS使用单字节8位动作（master_kfs_action_bits_0）
	 * bit0~1: 旋转三档 00/01/10（11预留）
	 * bit2   : 前臂二档 1/0
	 * bit3~5 : 主轴抬升七种状态 0~6（7预留）
	 * bit6~7 : 伸缩杆两位置 00/01（10/11预留=停止）
	 */
	if (control_mode == master_control)
	{
		uint16_t kfs_action_word = (uint16_t)master_kfs_action_bits_0 |
		                           ((uint16_t)master_kfs_action_bits_1 << 8);
		uint8_t action = (uint8_t)(kfs_action_word & 0xFFU);

		/* bit0~1: 三档旋转 => three_kfs_position */
		switch (action & 0x03U)
		{
			case 0: three_kfs_position = three_kfs_p1; break; /* 00 */
			case 1: three_kfs_position = three_kfs_p2; break; /* 01 */
			case 2: three_kfs_position = three_kfs_p3; break; /* 10 */
			default: three_kfs_position = three_kfs_p1; break; /* 11预留 */
		}

		/* bit2: 前臂两档 => kfs_spin_position */
		kfs_spin_position = ((action & (1U << 2)) != 0U) ? kfs_spin_p2 : kfs_spin_p1; /* 1/0 */

		/* bit3~5: 主轴抬升四种状态（用3位编码，先做简单状态机映射，不写具体执行）
		 * 001 -> 状态机1
		 * 010 -> 状态机2
		 * 011 -> 状态机3
		 * 100 -> 状态机4
		 * 其余值预留：保持当前状态不变
		 */
		{
			uint8_t lift_code = (uint8_t)((action >> 3) & 0x07U);
			switch (lift_code)
			{
				case 0x01U: main_lift_position = main_lift_p1; break; /* 001 */
				case 0x02U: main_lift_position = main_lift_p2; break; /* 010 */
				case 0x03U: main_lift_position = main_lift_p3; break; /* 011 */
				case 0x04U: main_lift_position = main_lift_p4; break; /* 100 */
				default:
					/* reserved */
					break;
			}
		}

		/* 伸缩两电机控制：
		 * 把两个字节拼成16位后取4位（两个电机各2位命令）
		 * - above电机命令位：bit7~6
		 * - below电机命令位：bit9~8（来自第二字节低2位）
		 *
		 * 命令编码：
		 * 00 不动
		 * 01 伸出 -> -100
		 * 10 收回 -> +100
		 * 11 预留（按不动处理）
		 */
		{
			uint8_t above_cmd = (uint8_t)((kfs_action_word >> 6) & 0x03U);
			uint8_t below_cmd = (uint8_t)((kfs_action_word >> 8) & 0x03U);

			if (above_cmd == 0x01U)
			{
				master_kfs_above_spd_cmd = -100;
			}
			else if (above_cmd == 0x02U)
			{
				master_kfs_above_spd_cmd = 100;
			}
			else
			{
				master_kfs_above_spd_cmd = 0;
			}

			if (below_cmd == 0x01U)
			{
				master_kfs_below_spd_cmd = -100;
			}
			else if (below_cmd == 0x02U)
			{
				master_kfs_below_spd_cmd = 100;
			}
			else
			{
				master_kfs_below_spd_cmd = 0;
			}
		}
	}

//通道一控制三个kfs旋转
	static uint16_t ch1_prev = 0;
	
	if (control_mode == remote_control)
	{
		if (RCctrl.CH1 == CH1_HIGH && ch1_prev != CH1_HIGH)
		{
			three_kfs_position = (Three_kfs_position)(((int)three_kfs_position + 1) % 6);
		}
		if (RCctrl.CH1 == CH1_LOW && ch1_prev != CH1_LOW)
		{
			three_kfs_position = (Three_kfs_position)(((int)three_kfs_position - 1+6) % 6);
		}
		ch1_prev = RCctrl.CH1;
	}
	
	
	


	float tar_3k;
	
	switch(three_kfs_position)
	{
		case three_kfs_p1:
			tar_3k = THREE_KFS_OFFSET1;
			// three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.70f, 0.26f, 0.0f);
			three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.86f, 0.28f, three_kfs_calc_feedforward_torque(three_kfs.position, tar_3k));
		break;
		case three_kfs_p2:
			tar_3k = THREE_KFS_OFFSET2;
			three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.86f, 0.28f, three_kfs_calc_feedforward_torque(three_kfs.position, tar_3k));
		break;
		case three_kfs_p3: 
			tar_3k = THREE_KFS_OFFSET3;
			three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.86f, 0.28f, three_kfs_calc_feedforward_torque(three_kfs.position, tar_3k));
		break;
		case three_kfs_p4:
			tar_3k = THREE_KFS_OFFSET4;
			three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.86f, 0.28f, three_kfs_calc_feedforward_torque(three_kfs.position, tar_3k));
		break;
		case three_kfs_p5:
			tar_3k = THREE_KFS_OFFSET5;
			three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.86f, 0.28f, three_kfs_calc_feedforward_torque(three_kfs.position, tar_3k));
		break;
		case three_kfs_p6:
			tar_3k = THREE_KFS_OFFSET6;
			three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.86f, 0.28f, three_kfs_calc_feedforward_torque(three_kfs.position, tar_3k));
		break;
		default: tar_3k = three_kfs_Initpos;
	}
	// three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.5f, 0.0f, 0.0f);
	// three_kfs.set_mit_data(&three_kfs, tar_3k, 0.0f, 0.0f, 0.0f, 0.0f);
	
//通道三控制主升机构升降
// 	static uint16_t ch3_prev = 0;
	
// 	if (control_mode == remote_control)
// 	{
// 		if (RCctrl.CH3 == CH3_HIGH && ch3_prev != CH3_HIGH)
// 		{
// 			main_lift_position = (Main_lift_position)(((int)main_lift_position + 1) % 4);
// 		}
// 		if (RCctrl.CH3 == CH3_LOW && ch3_prev != CH3_LOW)
// 		{
// 			main_lift_position = (Main_lift_position)(((int)main_lift_position - 1+4) % 4);
// 		}
// 		ch3_prev = RCctrl.CH3;
// 	}


// float tar_lift;
// 	switch(main_lift_position)
// 	{
// 		case main_lift_p1:
// 			tar_lift = MAIN_LIFT_OFFSET1;
// 			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.0f, 0.0f, 0.0f);
// 		break;
// 		case main_lift_p2:
// 			tar_lift = MAIN_LIFT_OFFSET2;
// 			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.30f, 0.025f, -2.0f);
// 		break;
// 		case main_lift_p3:
// 			tar_lift = MAIN_LIFT_OFFSET3;
// 			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.20f, 0.025f, -2.0f);
// 		break;
// 		case main_lift_p4:
// 			tar_lift = MAIN_LIFT_OFFSET4;
// 			main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.10f, 0.025f, -2.0f);
// 		break;		
// 		default: tar_lift = main_lift_Initpos;
// 	}
	// main_lift.set_mit_data(&main_lift, tar_lift, 0.0f, 0.2, 0.15f, -5.0f);
	main_lift.set_mit_data(&main_lift, 0, (992-RCctrl.CH3)/150, 0, 0.3, -1.0);
				
//通道四控制前臂kfs旋转


	static uint16_t ch4_prev = 0;

		if (control_mode == remote_control)
		{
			if (RCctrl.CH4 == CH4_HIGH && ch4_prev != CH4_HIGH)
			{
				kfs_spin_position = (Kfs_spin_position)(((int)kfs_spin_position + 1) % 2);
			}
			if (RCctrl.CH4 == CH4_LOW && ch4_prev != CH4_LOW)
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
			kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.1f, 0.4f, 0.0f);
		break;
		case kfs_spin_p3:
			// tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET3;
			// kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 11.0f, 2.6f, -4.2f);
		break;
		case kfs_spin_p4:
			// tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET4;
			// kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.2f, 1.0f, 0.0f);
		break;
		case kfs_spin_p5:
			// tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET5;
			// kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 11.0f, 2.6f, -4.2f);
		break;
		case kfs_spin_p6:
			// tar_spin = kfs_spin_Initpos + KFS_SPIN_OFFSET6;
			// kfs_spin.set_mit_data(&kfs_spin, tar_spin, 0.0f, 0.2f, 1.0f, 0.0f);
		break;
	}


	
	
//通道二控制伸缩
	

		if (control_mode == master_control)
		{
			kfs_above.PID_Calculate(&kfs_above, master_kfs_above_spd_cmd);
			kfs_below.PID_Calculate(&kfs_below, master_kfs_below_spd_cmd);
		}
		// CH5切换控制电机
		else if (control_mode == remote_control)
		{
			/* 从其他模式切回遥控时，同步上一拍输入，避免CH5边沿误触发 */
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
			/* 其他模式下清零，避免沿用上一拍残留输出 */
			kfs_above.PID_Calculate(&kfs_above, 0);
			kfs_below.PID_Calculate(&kfs_below, 0);
		}

		last_control_mode = control_mode;



 	DJIset_motor_data(&hfdcan3, 0X200, kfs_above.pid_spd.Output,kfs_below.pid_spd.Output,0.0f,0.0f);

}
