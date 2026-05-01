#include "Can_Task.h"
#include "Motion_Task.h"
#include "motor.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "chassis.h"
#include "kfs.h"
#include "lift.h"
#include "weapon.h"
#include "tim.h"
#include "remote_control.h"
#include "usart.h"
#include "bsp_can_tim.h"
void Can_Task(void const * argument)
{
    // TickType_t Systick = 0;
    uint32_t can1_free_level = 0;
    uint32_t can2_free_level = 0;
    uint32_t can3_free_level = 0;
   
    for(;;)
    {
        RemoteControl_LinkWatchdog_SimpleTest(&RCctrl);
#if REMOTE_LOST_PROTECT_ENABLE
        RemoteControl_LinkWatchdog_Update(&RCctrl);

        if (RCctrl.rc_lost != false)
        {
            /* ң����·��ʧ��ȫ����ض���� */
            Chassis.Chassis_Stop(&Chassis);
            DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

            R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

            osDelay(1);
            continue;
        }
#endif

        Motor_OverTemp_SimpleTest();

        if (Motor_OverTempProtect_Update() != 0U)
        {
            /* ���±�����ȫ���������㲢����������ҵ����� */
            Chassis.Chassis_Stop(&Chassis);
            DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
            DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

            R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

            osDelay(1);
            continue;
        }

        // Systick = osKernelGetTickCount();

//          if(Chassis.super_struct.base.error_code == 0x00)
//          {
				
						
             
//					}

//          if(Lift.super_struct.base.error_code == 0x00)
//          {
//              
//          }
//          if(Weapon.super_struct.base.error_code == 0x00)
//          {
//						
//          }
//	
		
		// if (RCctrl.rc_lost){
		// 	if(Systick % 2 == 1){	
        //          Chassis.Chassis_Stop(&Chassis);
		// 		DJIset_motor_data(&hfdcan1, 0X200,0,0,0,0);
		// 		DJIset_motor_data(&hfdcan2, 0X200,0,0,0,0);
		// 	}
		// 	if(Systick % 2 == 0){	
			
		// 	}
		// }
		// else{
            switch(control_mode)
            {
                case master_control:
                    /* ���е��ȣ���ģ�鰴enableλ�������У���ͬʱ��Ч */
                    if ((master_enable_bits & MASTER_EN_CHASSIS) != 0U)
                    {
                        manual_chassis_function();
                    }
                    if ((master_enable_bits & MASTER_EN_WEAPON) != 0U)
                    {
                        manual_weapon_function();
                    }
                    if ((master_enable_bits & MASTER_EN_LIFT) != 0U)
                    {
                        manual_lift_function();
                    }
                    if ((master_enable_bits & MASTER_EN_KFS) != 0U)
                    {
                        manual_kfs_function();
                    }
                    break;
                case emergency_stop_mode:
                    /* ��ͣģʽ������������������������������������� */
                    Chassis.Chassis_Stop(&Chassis);
                    DJIset_motor_data(&hfdcan1, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan2, 0X200, 0, 0, 0, 0);
                    DJIset_motor_data(&hfdcan3, 0X200, 0, 0, 0, 0);

                    /* DM�����MIT�����㣺kp/kd/torqueȫ0 */
                    R2_lift_motor_left.set_mit_data(&R2_lift_motor_left, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    R2_lift_motor_right.set_mit_data(&R2_lift_motor_right, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    main_lift.set_mit_data(&main_lift, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                    kfs_spin.set_mit_data(&kfs_spin, 0.0f, 0.9f, 0.3f, 0.4f, 0.0f);
                    three_kfs.set_mit_data(&three_kfs, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

                    /* ��ͣʱ��weapon���ִ�������س�ʼ����ƽ */
                    servo_state = 1U;
                    clamp_state = 0U;
                    sucker1_state = 0U;
                    sucker2_state = 0U;
                    sucker3_state = 0U;
                    sucker4_state = 0U;
//                    pump1_state = 0U;
//                    pump2_state = 0U;

                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2100);            /* �����ʼ��λ */
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);           /* ��צ��ʼ����ƽ */
//                    pump1_two_suckers_linkage_nominal_open(0U, 0U);                /* ����1/2���1��ʼ�� */
//                    pump2_two_suckers_linkage_nominal_open(0U, 0U);                /* ����3/4���2��ʼ�� */
                    break;
                case remote_control:
                    switch (remote_mode)
                    {
                        case chassis_mode:
                            manual_chassis_function();
                            break;
                        
                        case weapon_mode:
                            manual_weapon_function();
                            break;
                        
                        case lift_mode:
                            Chassis.Chassis_Stop(&Chassis);
                            // ֱ�ӷ�0��ɲ����
                            DJIset_motor_data(&hfdcan1, 0x200, 0, 0, 0, 0);
                            manual_lift_function();
                            break;
                        case kfs_mode:
                            Chassis.Chassis_Stop(&Chassis);
                            // ֱ�ӷ�0��ɲ����
                            DJIset_motor_data(&hfdcan1, 0x200, 0, 0, 0, 0);
                            manual_kfs_function();
                            break;
                        case remote_none:
                            break;
                    }
                    break;
            }

//			if(Systick % 10 == 0){	
//                
//                
//			}
//			if(Systick % 10 == 5){	
//                
//                
//			}

		// }
        can1_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        can2_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
		    can3_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3);

        if (can1_send_flag)
        {
            can1_send_flag = 0;
            DJIset_motor_data(&hfdcan1, 0X200, 
                chassis_motor1.pid_spd.Output, 
                chassis_motor2.pid_spd.Output,
                chassis_motor3.pid_spd.Output, 
                chassis_motor4.pid_spd.Output);
			// DM电机发送（FDCAN1）
			R2_lift_motor_left.send_mit_data(&R2_lift_motor_left);
			R2_lift_motor_right.send_mit_data(&R2_lift_motor_right);
        }
        if (can2_send_flag)
        {
            can2_send_flag = 0;
            DJIset_motor_data(&hfdcan2, 0X200, 
                guide_motor1.pid_spd.Output, 
                guide_motor2.pid_spd.Output,
                flexible_motor1.pid_spd.Output, 
                flexible_motor2.pid_spd.Output);
			// DM电机发送（FDCAN2）
			main_lift.send_mit_data(&main_lift);
        }
        if (can3_send_flag)
        {
            can3_send_flag = 0;
            DJIset_motor_data(&hfdcan3, 0X200, 
                kfs_above.pid_spd.Output, 
                kfs_below.pid_spd.Output,
                0, 0);
			// DM电机发送（FDCAN3）
			kfs_spin.send_mit_data(&kfs_spin);
			three_kfs.send_mit_data(&three_kfs);
        }

		osDelay(1);
    }

}


