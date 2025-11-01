#include "include.h"

static u8 key_num = 0;

/******************************************
*Name:		gerKey
*Function:	判断按键是否按下(determine if the button is pressed)
*Input:		null
*Return:	TRUE/FALSE
*Author:    ROC
******************************************/
bool gerKey(void)
{
	if(!KEY)
	{
		DelayMs(10);
		if(!KEY)
		{
			return TRUE;
		}
		return FALSE;
	}
	else
		return FALSE;
}

/******************************************
*Name:		Control
*Function:	通过按键来转换行驶模式(switch the driving mode via the button)
*Input:		null
*Return:	null
*Author:    ROC
*Date:		2018/05/30
******************************************/
void Control(void)
{
	
	if(gerKey())
	{
		key_num++;
			if(key_num > 4)
				key_num = 0;
		
		LED=0;
		DelayMs(1000);
		LED=1;
		
	}
	switch(key_num)
	{
		case 0:
		MotorControl(0,0);   //前进(forward)
		break;
		
		case 1:
		MotorControl(-100,-100);   //前进(forward)
		break;
		
		case 2:
		MotorControl(100,100);   //后退(backward)
		break;
		
		case 3:
		MotorControl(100,-100);   //左转(turn left)
		break;
		
	  case 4:
		MotorControl(-100,100);   //右转(turn right)
		break;
				
		default:
		key_num=0;
		break;
	}
}

