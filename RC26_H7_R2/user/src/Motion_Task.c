#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "weapon.h"
#include "usbd_cdc_if.h"

//Weapon_mode weapon_mode;
Control_mode control_mode;
Remote_mode remote_mode;
Master_mode master_mode;
uint8_t master_weapon_action_bits;
R2_lift_mode r2_lift_mode;

static uint8_t rc_bit_minmax_decode(uint16_t ch_val)
{
    if (ch_val <= 500u) return 0u;
    if (ch_val >= 1500u) return 1u;
    return 2u;}


void Motion_Task(void const * argument)
{

  for(;;)
  {
		uint8_t ch6_bit = rc_bit_minmax_decode(RCctrl.CH6); 
		uint8_t ch7_bit = rc_bit_minmax_decode(RCctrl.CH7);
		uint8_t mode_code = (uint8_t)((ch6_bit << 1) | ch7_bit);
	
		
		
		
		
		
		if(RCctrl.CH8 < 500)
		{
			control_mode  = master_control;
		}
		
		else if(RCctrl.CH8 > 500 && RCctrl.CH8 < 1500)
		{
			control_mode  = part_remote_control;
		}
		
		else
		{
			control_mode  = remote_control;

		}
		
		
		
		
		switch(control_mode)
			{
						
				case remote_control:
//00;底盘 01;武器 10;抬升 11;kfs
          if ((ch6_bit <= 1u) && (ch7_bit <= 1u))
          {
              switch (mode_code)
              {
                  case 0u: // 00
                      remote_mode = chassis_mode;
                      break;
                  case 1u: // 01
											remote_mode = weapon_mode;
                      break;
                  case 2u: // 10
											remote_mode = lift_mode;
                      break;
                  case 3u: // 11
											remote_mode = kfs_mode;
                      break;
                  default:
                      break;
              }
          }
					break;
					
				case part_remote_control:
					break;
				case master_control:
          /* 主控模式：使用USB数据包data[0]的位控制任务状态机
           * bit0=底盘 bit1=武器 bit2=抬升 bit3=kfs
           * 多个位同时为1时按底盘>武器>抬升>kfs优先级取一个
           */
          if (usb_last_packet_valid != 0U)
          {
            uint8_t master_bits = usb_last_packet_data[0];
            master_weapon_action_bits = usb_last_packet_data[1];

            if ((master_bits & 0x01U) != 0U)
            {
              master_mode = master_chassis_mode;
            }
            else if ((master_bits & 0x02U) != 0U)
            {
              master_mode = master_weapon_mode;
            }
            else if ((master_bits & 0x04U) != 0U)
            {
              master_mode = master_lift_mode;
            }
            else if ((master_bits & 0x08U) != 0U)
            {
              master_mode = master_kfs_mode;
            }
            else
            {
              master_mode = master_none;
            }
          }
					break;
			}
		
		}
      
      
    osDelay(1);

  }

