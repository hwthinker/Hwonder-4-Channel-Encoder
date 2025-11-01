#include "include.h"

void I2C_SDA_OUT(void)//SDA输出方向配置(SDA output direction configuration)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	//RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
	GPIO_InitStructure.GPIO_Pin=IIC_IO_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//SDA推挽输出(SDA push-pull output)
	GPIO_Init(GPIOX,&GPIO_InitStructure); 						
}

void I2C_SDA_IN(void)//SDA输入方向配置(SDA input direction configuration)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	//RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
	GPIO_InitStructure.GPIO_Pin=IIC_IO_SDA;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//SCL上拉输入(SCL pull-up input)
	GPIO_Init(GPIOX,&GPIO_InitStructure);
}

//以下为模拟IIC总线函数(The following are functions for simulating IIC bus)
void IIC_init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(CLOCK, ENABLE);	 //使能PD端口时钟(Enable PD port clock)
	GPIO_InitStructure.GPIO_Pin = IIC_IO_SDA |IIC_IO_SCL;	//PD6配置为推挽输出,SCL(PD6 is configured as push-pull output, SCL)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz(IO port speed is 50MHz)
	GPIO_Init(GPIOX, &GPIO_InitStructure);					 //根据设定参数初始化GPIOD(Initialize GPIOD according to the set parameters)
	GPIO_SetBits(GPIOX,IIC_IO_SDA|IIC_IO_SCL); 
}

void IIC_start()	//起始信号(Start signal)
{
	I2C_SDA_OUT();
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	DelayUs(5);
	IIC_SDA=0;
	DelayUs(5);
	IIC_SCL=0;
}

void IIC_stop()		//终止信号(Stop signal)
{
	I2C_SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
	DelayUs(5);
	IIC_SCL=1; 
	IIC_SDA=1;
	DelayUs(5);
}

//主机产生一个应答信号(Master generates an acknowledge signal)
void IIC_ack()
{
	IIC_SCL=0;
	I2C_SDA_OUT();
  IIC_SDA=0;	
  DelayUs(2);
  IIC_SCL=1;
  DelayUs(2);
  IIC_SCL=0;	
}

//主机不产生应答信号(Master does not generate an acknowledge signal)
void IIC_noack()
{
	IIC_SCL=0;
	I2C_SDA_OUT();
  IIC_SDA=1;
  DelayUs(2);
  IIC_SCL=1;
  DelayUs(2);
  IIC_SCL=0;
}

//等待从机应答信号(Wait for slave to acknowledge signal)
//返回值：1 接收应答失败(Return value: 1 receive acknowledgment failed)
//		  0 接收应答成功(0 receive acknowledgment succeeded)
u8 IIC_wait_ack()
{
	u8 tempTime=0;
	I2C_SDA_IN();
	IIC_SDA=1;
	DelayUs(1);
	IIC_SCL=1;
	DelayUs(1);

	while(READ_SDA)
	{
		tempTime++;
		if(tempTime>250)
		{
			IIC_stop();
			return 1;
		}	 
	}

	IIC_SCL=0;
	return 0;
}

void IIC_send_byte(u8 txd)
{
	u8 i=0;
	I2C_SDA_OUT();
	IIC_SCL=0;;//拉低时钟开始数据传输(Pull down the clock to start data transmission)
	for(i=0;i<8;i++)
	{
		IIC_SDA=(txd&0x80)>>7;//读取字节(Read byte)
		txd<<=1;
		DelayUs(2);
		IIC_SCL=1;
		DelayUs(2); //发送数据(Send data)
		IIC_SCL=0;
		DelayUs(2);
	}
}



//读取一个字节(Read a byte)
u8 IIC_read_byte(u8 ack)
{
	u8 i=0,receive=0;
	I2C_SDA_IN();
  for(i=0;i<8;i++)
  {
   	IIC_SCL=0;
		DelayUs(2);
		IIC_SCL=1;
		receive<<=1;//左移(Left shift)
		if(READ_SDA)
		   receive++;//连续读取八位(Read eight bits continuously)
		DelayUs(1);	
  }

  if(!ack)
	  IIC_noack();
	else
		IIC_ack();

	return receive;//返回读取到的字节(Return the byte read)
}
