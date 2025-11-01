#ifndef _ADC_H_
#define _ADC_H_

#define ADC_BAT		13		//电池电压的AD检测通道(AD detection channel for battery voltage)
#define BUZZER 		PCout(9)
#define LED PCout(15)
#define KEY PCin(0)		//按键(button)

void InitADC(void);
void InitBuzzer(void);
void InitLED(void);
void InitKey(void);

void CheckBatteryVoltage(void);
u16 GetBatteryVoltage(void);
void Buzzer(void);
uint16 GetADCResult(BYTE ch);


#endif
