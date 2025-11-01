/******************************************************
//	Company:深圳幻尔科技有限公司(Shenzhen Hiwonder Technology Co., Ltd.)										   //	
//	我们的店铺(Online Shop):lobot-zone.taobao.com                  //
//	我们的官网(Official website)：www.lobot-robot.com                   //
******************************************************/


#include "include.h"

#define CAM_DEFAULT_I2C_ADDRESS       (0x34)			//I2C地址(I2C address)
#define MOTOR_TYPE_ADDR               0x14 				//编码电机类型设置寄存器地址(Encoder motor type setting register address)
#define MOTOR_FIXED_SPEED_ADDR       	0x33				//速度寄存器地址。属于闭环控制(Speed register address; belongs to closed-loop control)
#define MOTOR_ENCODER_POLARITY_ADDR   0x15				//电机编码方向极性地址(Motor encoder direction polarity address)
#define MOTOR_FIXED_PWM_ADDR      		0x1F				//固定PWM控制地址，属于开环控制(Fixed PWM control address, belongs to open-loop control)
#define MOTOR_ENCODER_TOTAL_ADDR  		0x3C				//4个编码电机各自的总脉冲值(Total pulse value of each of the 4 encoding motors)
#define ADC_BAT_ADDR                  0x00				//电压地址(Voltage address)


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

//电机类型具体地址(motor type specific address)
#define MOTOR_TYPE_WITHOUT_ENCODER        0 		//无编码器的电机,磁环每转是44个脉冲减速比:90  默认(Motor without encoder, 44 pulses per magnetic ring rotation, reduction ratio: 90, default)
#define MOTOR_TYPE_TT                     1 		//TT编码电机(TT encoder motor)
#define MOTOR_TYPE_N20                    2 		//N20编码电机(N20 encoder motor)
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3 		//磁环每转是44个脉冲   减速比:90  默认(44 pulses per magnetic ring rotation, reduction ratio: 90, default)

/*用数组传递电机速度，正数为设置前进速度，负数为设置后退速度(Use an array to pass motor speed, positive number is set to forward speed, negative number is set to reverse speed)
  以p1、p2为例：p1=4个电机以50的速度前进    p2=4个电机以20的速度后退(Take p1 and p2 as examples: p1 = 4 motors with a speed of 50 forward    p2 = 4 motors with a speed of 20 backward)*/
uint8_t p1[4]={50,50,50,50};      										        //电机转速设置(set motor speed)                       
uint8_t p2[4]={-20,-20,-20,-20};    									    //电机转速设置(set motor speed) 
int8_t stop[4]={0,0,0,0}; 							                    //电机转速设置(set motor speed)
int8_t EncodeReset[16]={0};                                         //用于重置脉冲值(used to reset the pulse width)
int8_t MotorEncoderPolarity = 0;                                      //电机极性控制变量(motor polarity control variable)
uint32_t EncodeTotal[4];													    //用于暂存电机累积转动量的值，正转递增，反转递减(Used to temporarily store the accumulated rotation of the motor, increasing when rotating forward, decreasing when rotating backward)
int8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM; 								        //设置电机类型(set motor type)
uint8_t data[3];                                                                        //用于暂存电压ADC数据(used to temporarily store voltage ADC data)
	
	
int main(void)
{
	u16 v;																  						//用于暂存电压值(Used to temporarily store voltage)
	SystemInit(); 																			//系统时钟初始化为72M	  SYSCLK_FREQ_72MHz(System clock is initialized to 72M   SYSCLK_FREQ_72MHz)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//设置NVIC中断分组2:2位抢占优先级，2位响应优先级(Set NVIC interrupt group 2: 2-bit preemption priority, 2-bit response priority)
	InitDelay(72); 																			//初始化延时函数(Initialize delay function)
	InitTimer2();																				//定时器2初始化(Timer 2 initialization)
	IIC_init();																					//IIC初始化(IIC initialization)
	Usart1_Init(); 																			//串口初始化(Serial port initialization)
	InitLED();        																	//初始化LED指示灯(Initialize LED indicator)
	DelayMs(200);
	I2C_Write_Len(MOTOR_TYPE_ADDR,&MotorType,1);					//在电机类型地址中写入电机类型编号(Write the motor type number to the motor type address)
	DelayMs(5);
	I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR,&MotorEncoderPolarity,1);		//设置电机极性(Set motor polarity)
	DelayMs(5);
	while(1)
	{
		I2C_Read_Len(ADC_BAT_ADDR,data,2);									//读取电机电压(Read motor voltage)
		v = data[0] + (data[1]<<8); 												//转换电压(Convert voltage)
		printf("V = ");		printf("%d",v);  	printf("mV\n"); //打印电压(Print voltage)
		
		/*在电机转速地址中写入电机的转动方向和速度：WireWriteDataArray（转速控制地址，电机转速数组，电机个数）(Write the motor rotation direction and speed to the motor speed address: WireWriteDataArray (speed control address, motor speed array, and number of motors))*/
		I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p1,4);						//控制电机转动(Control motor rotation)
		DelayMs(3000);
		I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,p2,4);						//控制电机转动(Control motor rotation)
		DelayMs(3000);
		
		
		//I2C_Write_Len(MOTOR_FIXED_PWM_ADDR,p1,4);					//PWM控制(PWM控制程序中不能有延时，否则会停止电机运行，打印也有一定的延时)(PWM control (There should be no delay in the PWM control program; otherwise, it will interrupt motor operation, and printing also introduces some delays))	
		//DelayMs(3000);
		
		I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);
		printf("Encode1 = %ld  Encode2 = %ld  Encode3 = %ld  Encode4 = %ld  \r\n", EncodeTotal[0], EncodeTotal[1], EncodeTotal[2], EncodeTotal[3]);
		
		
		I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR,stop,4);						//控制电机停止(control the motor to stop)
		DelayMs(1000);
		
		I2C_Write_Len(MOTOR_ENCODER_TOTAL_ADDR,EncodeReset,16);		//重置脉冲值(reset pulse value)
		I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);
		printf("Encode1 = %ld  Encode2 = %ld  Encode3 = %ld  Encode4 = %ld  \r\n", EncodeTotal[0], EncodeTotal[1], EncodeTotal[2], EncodeTotal[3]);
		DelayMs(3000);
	}
}

