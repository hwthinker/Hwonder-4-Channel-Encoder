#include "include.h"


static u8  fac_us=0;//us延时倍乘数(us delay multiplier)
static u16 fac_ms=0;//ms延时倍乘数(ms delay multiplier)
//初始化延迟函数(Initialize delay function)
//SYSTICK的时钟固定为HCLK时钟的1/8(The clock of SYSTICK is fixed to 1/8 of the HCLK clock)
//SYSCLK:系统时钟(system clock)
void InitDelay(u8 SYSCLK)
{
//	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8(bit2 clear; select external clock HCLK/8)
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8(Select external clock HCLK/8)
	fac_us=SYSCLK/8;
	fac_ms=(u16)fac_us*1000;
}
//延时nms(Delay nms)
//注意nms的范围(Note the range of nms)
//SysTick->LOAD为24位寄存器,所以,最大延时为:(SysTick->LOAD is a 24-bit register, so the maximum delay is:)
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms(SYSCLK is in Hz, and nms is in ms)
//对72M条件下,nms<=1864(Under 72M conditions, nms<=1864)
void DelayMs(u16 nms)
{
	u32 temp;
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)(Load time (SysTick->LOAD is 24bit))
	SysTick->VAL =0x00;           //清空计数器(Clear counter)
	SysTick->CTRL=0x01 ;          //开始倒数(Start countdown)
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达(Wait for time to arrive)
	SysTick->CTRL=0x00;       //关闭计数器(Close counter)
	SysTick->VAL =0X00;       //清空计数器(Clear counter)
}
//延时nus(Delay nus)
//nus为要延时的us数.(nus is the number of us to delay)
void DelayUs(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us; //时间加载(Load time)
	SysTick->VAL=0x00;        //清空计数器(Clear counter)
	SysTick->CTRL=0x01 ;      //开始倒数(Start countdown)
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达(Wait for time to arrive)
	SysTick->CTRL=0x00;       //关闭计数器(Close counter)
	SysTick->VAL =0X00;       //清空计数器(Clear counter)
}
