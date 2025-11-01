#include "include.h"


#pragma import(__use_no_semihosting)             
//标准库需要的支持函数(the required support function for standard library)                
struct __FILE 
{ 
	int handle; 
}; 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式(define the _sys_exit function to avoid entering semi-hosting mode)    
void _sys_exit(int x) 
{ 
	x = x; 
} 
///重定义printf到USART1(redefine the printf function to USART1)
int fputc(int ch, FILE *f)
{
USART_SendData(USART1, (uint8_t) ch);
while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
return (ch);
}


void Usart1_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART 初始化设置(USART initialization setting)

	USART_InitStructure.USART_BaudRate = 9600;//一般设置为9600(usually set to 9600);
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断(start interrupt)

	USART_Cmd(USART1, ENABLE);                    //使能串口(enable the serial port)
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能(enable IRQ channel)
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1(initialize the NVIC register of the USART1 peripheral based on the parameters specified in the NVIC_InitStruc)
}

void USART1_IRQHandler(void)//串口输出printf();(serial output printf())
{ //Do you want to do	
	
// unsigned char ch;
// if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET) 
// ch = USART_ReceiveData(USART1); 
// printf("%c",ch); 
}
