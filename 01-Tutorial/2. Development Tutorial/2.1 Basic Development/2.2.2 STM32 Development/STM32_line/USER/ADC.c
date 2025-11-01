#include "include.h"

uint8 BuzzerState = 0;
uint16 BatteryVoltage;

void InitLED(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB端口时钟(Enable PB port clock)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出(Push-pull output)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}
void InitBuzzer(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能端口时钟(Enable port clock)

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出(Push-pull output)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void InitADC(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE);	   //使能ADC1通道时钟(Enable ADC1 channel clock)

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //72M/6=12,ADC最大时间不能超过14M(72M/6=12, the maximum conversion time of ADC cannot exceed 14M)
	//PA0/1/2/3 作为模拟通道输入引脚(PA0/1/2/3 is used as the analog channel input pin)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚(Analog input pin)
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值(Reset all registers of peripheral ADC1 to default values)

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式(ADC works in independent mode for ADC1 and ADC2)
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式(The analog-to-digital conversion works in single-channel mode)
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式(The analog-to-digital conversion works in single conversion mode)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动(Conversion is started by software rather than external trigger)
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐(ADC data is right-aligned)
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目(Number of ADC channels for sequential regular conversion)
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器(Initialize the registers of peripheral ADCx according to the parameters specified in ADC_InitStruct)



	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1(Enable specified ADC1)

	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器(Reset the calibration register of the specified ADC1)

	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待(Get the status of the ADC1 reset calibration register, wait if the status is set)

	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态(Start the calibration status of the specified ADC1)

	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待(Get the calibration program of the specified ADC1, wait if the status is set)

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能(Enable the software conversion start function of the specified ADC1)
}


uint16 GetADCResult(BYTE ch)
{
	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间(Set the conversion sequence of the specified ADC channel group, their conversion order, and sampling time)
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期(ADC1, ADC channel 3, the regular sampling order value is 1, and the sampling time is 239.5 cycles)

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能(Enable the software conversion start function of the specified ADC1)

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //等待转换结束(Wait for conversion to complete)

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果(Return the conversion result of the most recent ADC1 regular group)
}


void CheckBatteryVoltage(void)
{
	uint8 i;
	uint32 v = 0;
	for(i = 0;i < 8;i++)
	{
		v += GetADCResult(ADC_BAT);
	}
	v >>= 3;
	
	v = v * 2475 / 1024;//adc / 4096 * 3300 * 3(3代表放大3倍，因为采集电压时电阻分压了)((3 represents amplification by 3 times, because the voltage was divided by resistance when collecting voltage))
	BatteryVoltage = v;

}

uint16 GetBatteryVoltage(void)
{//电压毫伏(Voltage in millivolts)
	return BatteryVoltage;
}

void Buzzer(void)
{//放到100us的定时中断里面(Put it in the 100us timer interrupt)
	static bool fBuzzer = FALSE;
	static uint32 t1 = 0;
	static uint32 t2 = 0;
	if(fBuzzer)
	{
		if(++t1 >= 2)
		{
			t1 = 0;
			BUZZER = !BUZZER;//2.5KHz
		}
	}
	else
	{
		BUZZER = 0;
	}

	
	if(BuzzerState == 0)
	{
		fBuzzer = FALSE;
		t2 = 0;
	}
	else if(BuzzerState == 1)
	{
		t2++;
		if(t2 < 5000)
		{
			fBuzzer = TRUE;
		}
		else if(t2 < 10000)
		{
			fBuzzer = FALSE;
		}
		else
		{
			t2 = 0;
		}
	}
}


