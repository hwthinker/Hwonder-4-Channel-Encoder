#include "include.h"

#define CAM_DEFAULT_I2C_ADDRESS       (0x34)			//I2C地址(I2C address)
#define MOTOR_TYPE_ADDR               20 				//编码电机类型设置寄存器地址(Encoder motor type setting register address)
#define MOTOR_FIXED_SPEED_ADDR       	51				//速度寄存器地址。属于闭环控制(Speed register address; belongs to closed-loop control)
#define MOTOR_ENCODER_POLARITY_ADDR   21				//电机编码方向极性地址(Motor encoder direction polarity address)
#define MOTOR_FIXED_PWM_ADDR      		31				//固定PWM控制地址，属于开环控制(Fixed PWM control address, belongs to open-loop control)
#define MOTOR_ENCODER_TOTAL_ADDR  		60				//4个编码电机各自的总脉冲值(Total pulse value of each of the 4 encoding motors)
#define ADC_BAT_ADDR                  0					//电压地址(Voltage address)




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

uint8_t data[3]; 																//用于暂存电压ADC数据(used to temporarily store voltage ADC data)


//读取多个字节的数据（Reg：地址  Buf：数据内容 Len：数据长度）(Read multiple bytes of data (Reg: address  Buf: data content  Len: data length))
uint8_t I2C_Read_Len(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	IIC_start();																				//发送起始信号(send start signal)
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);	//发送元器件地址+写指令(Send device address and write command)
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1(Wait for the response. If it fails, send stop signal and return 1)
	{
		IIC_stop();					
		return 1;					
	}
	IIC_send_byte(Reg);																	//发送寄存器地址(send register address)
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1(Wait for the response. If it fails, send stop signal and return 1)
	{
		IIC_stop();
		return 1;
	}
	IIC_start();																				//发送起始信号(send start signal)
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 1);	//发送元器件地址+读指令(Send device address and read command)
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1(Wait for the response. If it fails, send stop signal and return 1)
	{
		IIC_stop();
		return 1;
	}
	for(i=0;i<Len;i++)																	//for循环Len次读取(Read Len times in a for loop)
	{	
		if(i != Len-1)																		//如果不是最后一次(If not the last time)
		{
			Buf[i] = IIC_read_byte(1);											//保存第I次的数据到数组的第I位，并发送答应信号(Save the data of the Ith time to the Ith position of the array and send the acknowledgment signal)
		}
		else
			Buf[i] = IIC_read_byte(0);											//保存第I次的数据到数组的第I位，并发送非答应信号(Save the data of the Ith time to the Ith position of the array and send a non-acknowledgment signal)
	}
	IIC_stop();																					//发送停止信号(Send stop signal)
	return 0;																						//读取成功，返回0(Read successfully, return 0)
}

//循环发送一个数组的数据（addr：地址  buf：数据内容  leng：数据长度）(Loop to send an array of data (addr: address  buf: data content  leng: data length))
uint8_t I2C_Write_Len(uint8_t Reg,uint8_t *Buf,uint8_t Len)//I2C的写数据(I2C write data)
{
	uint8_t i;
	IIC_start();																				//在起始信号后必须发送一个7位从机地址+1位方向位，用“0”表示主机发送数据，“1”表示主机接收数据。(After the start signal, a 7-bit slave address and 1-bit direction bit must be sent, with "0" indicating that the host sends data and "1" indicating that the host receives data)
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);	//发送 器件地址+写的命令(Send device address and write command)
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1(Wait for the response. If it fails, send stop signal and return 1)
	{
		IIC_stop();					
		return 1;					
	}
	IIC_send_byte(Reg);																	//发送 寄存器地址(send register address)
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1(Wait for the response. If it fails, send stop signal and return 1)
	{
		IIC_stop();
		return 1;
	}
	for(i =0;i<Len;i++)																	//循环 len 次写数据(Loop len times to write data)
	{
		IIC_send_byte(Buf[i]);														//发送第i位的8位数据(Send the 8-bit data of the i-th bit)
		if(IIC_wait_ack() == 1)														//等待响应，如果失败，发送停止信号，返回1(Wait for the response. If it fails, send stop signal and return 1)
		{
			IIC_stop();
			return 1;
		}
	}
	IIC_stop();																					//发送结束，发送停止信号(Send stop signal)
	return 0;																						//返回 0，确定发送成功(Return 0 to confirm successful transmission)
}

//循环发送一个数组的数据（addr：地址  buf：数据内容  leng：数据长度）
int8_t I2C_Write_Len(int8_t Reg,int8_t *Buf,int8_t Len)//I2C的写数据
{
	uint8_t i;
	IIC_start();																				//在起始信号后必须发送一个7位从机地址+1位方向位，用“0”表示主机发送数据，“1”表示主机接收数据。
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);	//发送 器件地址+写的命令
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1
	{
		IIC_stop();					
		return 1;					
	}
	IIC_send_byte(Reg);																	//发送 寄存器地址
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1
	{
		IIC_stop();
		return 1;
	}
	for(i =0;i<Len;i++)																	//循环 len 次写数据
	{
		IIC_send_byte(Buf[i]);														//发送第i位的8位数据
		if(IIC_wait_ack() == 1)														//等待响应，如果失败，发送停止信号，返回1
		{
			IIC_stop();
			return 1;
		}
	}
	IIC_stop();																					//发送结束，发送停止信号
	return 0;																						//返回 0，确定发送成功
}

//电机类型具体地址(motor type specific address)
#define MOTOR_TYPE_WITHOUT_ENCODER        0 		//无编码器的电机,磁环每转是44个脉冲减速比:90  默认(Motor without encoder, 44 pulses per magnetic ring rotation, reduction ratio: 90, default)
#define MOTOR_TYPE_TT                     1 		//TT编码电机(TT encoder motor)
#define MOTOR_TYPE_N20                    2 		//N20编码电机(N20 encoder motor)
#define MOTOR_TYPE_JGB                    3 		//磁环每转是44个脉冲   减速比:90  默认(44 pulses per magnetic ring rotation, reduction ratio: 90, default)

int8_t p0[4]={0,0,0,0};      										 //停止(stop)
int8_t p1[4]={-25,0,25,0};      						  //前进(forward)
int8_t p2[4]={25,0,-25,0};      							//后退(backward)


int8_t MotorEncoderPolarity = 0; 							  //电机极性控制变量(motor polarity control variable)
int32_t EncodeTotal[4];													//用于暂存电机累积转动量的值，正转递增，反转递减(Used to temporarily store the accumulated rotation of the motor, increasing when rotating forward, decreasing when rotating backward)
int8_t MotorType = MOTOR_TYPE_JGB; 								//设置电机类型(set motor type)
	
int main(void)
{
	static u8 key_num = 1 ;
	SystemInit(); 																			//系统时钟初始化为72M	  SYSCLK_FREQ_72MHz(System clock is initialized to 72M   SYSCLK_FREQ_72MHz)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//设置NVIC中断分组2:2位抢占优先级，2位响应优先级(Set NVIC interrupt group 2: 2-bit preemption priority, 2-bit response priority)
	InitDelay(72); 																			//初始化延时函数(Initialize delay function)
	InitTimer2();																				//定时器2初始化(Timer 2 initialization)
	IIC_init();																					//IIC初始化(IIC initialization)
	Usart1_Init(); 																			//串口初始化(Serial port initialization)
	InitLED();        																	//初始化LED指示灯(Initialize LED indicator)
	InitBusServoCtrl();                                 //总线舵机初始化(initialize bus servo)
	DelayMs(200);
	I2C_Write_Len(MOTOR_TYPE_ADDR,&MotorType,4);					//在电机类型地址中写入电机类型编号(Write the motor type number to the motor type address)
	DelayMs(5);
	I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR,&MotorEncoderPolarity,1);		//设置电机极性设置(Set motor polarity)
	DelayMs(5);
  while(1)
	{
		DelayMs(1000);
	  while(gerKey())
		{		
			if(key_num > 1)
				{
				key_num = 0;
				 }
			LED=0;
			DelayMs(1000);
			LED=1;
			TaskTimeHandle();  //ADC检测(ADC detection)
			switch(key_num)
			{
				case 0:
				//暂停(stop)
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				DelayMs(1800);			
				break;
				
				case 1:
				DelayMs(3600);
				//前进(forward)
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p1,4);
				DelayMs(1800);
				
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				DelayMs(1800);
				
				//后退(backward)
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p2,4);						
				DelayMs(1800);			
				
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				DelayMs(1800);
				
				//左转(turn left)
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,650,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p1,4);		
				DelayMs(1800);
				
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				DelayMs(1800);
				
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,650,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p2,4);						
				DelayMs(1800);
				
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				DelayMs(1800);
				
				//右转(turn right)
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,350,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p1,4);		
				DelayMs(1800);
				
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				DelayMs(1800);
				
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,350,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p2,4);						
				DelayMs(1800);
				
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,1000);
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				DelayMs(1800);
				
				//暂停(stop)
				I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p0,4);		
				BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,100);
				key_num = 0;
				break;
				
				default:
				break;
			}
			key_num++;
		}
 }
}
