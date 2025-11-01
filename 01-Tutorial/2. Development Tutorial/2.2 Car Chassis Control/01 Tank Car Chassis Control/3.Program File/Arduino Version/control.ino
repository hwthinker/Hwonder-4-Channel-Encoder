#include <Wire.h>

#define I2C_ADDR        0x34

#define ADC_BAT_ADDR                  0x00
#define MOTOR_TYPE_ADDR               0x14 //编码电机类型设置(Set encoder motor type)
#define MOTOR_ENCODER_POLARITY_ADDR   0x15 //设置编码方向极性(Set the encoder direction polarity)，
//如果发现电机转速根本不受控制，要么最快速度转动，要么停止。可以将此地址的值重新设置一下(If the motor speed is completely uncontrollable, either rotating at the fastest speed or stopping. You can reset the value of this address)
//范围0或1，默认0(Range 0 or 1, default 0)
#define MOTOR_FIXED_PWM_ADDR      0x1F //固定PWM控制，属于开环控制,范围（-100~100）(Fixed PWM control, belongs to open-loop control, range from -100 to 100)
//#define SERVOS_ADDR_CMD 40        
#define MOTOR_FIXED_SPEED_ADDR    0x33 //固定转速控制，属于闭环控制(Fixed speed control, belongs to closed-loop control)，
//单位：脉冲数每10毫秒，范围（根据具体的编码电机来，受编码线数，电压大小，负载大小等影响，一般在±50左右）(Unit: pulse count per 10 milliseconds, range (depending on the specific encoder motor, affected by the number of encoder lines, voltage size, load size, etc., generally around ±50))

#define MOTOR_ENCODER_TOTAL_ADDR  0x3C //4个编码电机各自的总脉冲值(Total pulse value of each of the four encoder motors)
//如果已知电机每转一圈的脉冲数为U，又已知轮子的直径D，那么就可以通过脉冲计数的方式得知每个轮子行进的距离(If the pulse count per revolution of the motor is known to be U, and the diameter of the wheel D is known, then the distance traveled by each wheel can be obtained through pulse counting)
//比如读到电机1的脉冲总数为P，那么行进的距离为(P/U) * (3.14159*D)(For example, if the total pulse count of motor 1 is P, then the distance traveled is (P/U) * (3.14159*D))
//对于不同的电机可以自行测试每圈的脉冲数U，可以手动旋转10圈读出脉冲数，然后取平均值得出(For different motors, you can test the number of pulses per revolution U by manually rotating 10 times and reading the pulse count, and then take the average value to obtain it)


//电机类型具体值(motor type specific values)
#define MOTOR_TYPE_WITHOUT_ENCODER        0
#define MOTOR_TYPE_TT                     1
#define MOTOR_TYPE_N20                    2
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3 //磁环每转是44个脉冲   减速比:90  默认(Magnetic ring rotates 44 pulses per revolution, reduction ratio: 90, default)

u8 data[20];
bool WireWriteByte(uint8_t val)   //通过I2C发送字节数据(Send byte data through I2C)
{
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) {
        return false;
    }
    return true;
}
bool WireWriteDataArray(  uint8_t reg,uint8_t *val,unsigned int len)    //通过I2C发送数据(Send data through I2C)
{
    unsigned int i;

    Wire.beginTransmission(I2C_ADDR);   //设置任务开始地址(Set the starting address of the task)
    Wire.write(reg);    //头地址内容写入(Write the header address)
    for(i = 0; i < len; i++) {
        Wire.write(val[i]);   //传入的内容写入I2C(Write the content passed in to I2C)
    }
    if( Wire.endTransmission() != 0 ) {     //写入是否联通判断(Determine if the write is connected)
        return false;
    }

    return true;
}

uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;    //电机模式设置(Motor mode setting)
uint8_t MotorEncoderPolarity = 0;     //电机极性设置(Motor polarity setting)
void setup()
{
  Wire.begin();
  delay(200);
  WireWriteDataArray(MOTOR_TYPE_ADDR,&MotorType,1);
  delay(5);
  WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR,&MotorEncoderPolarity,1);

}
int8_t car_forward[4]={0,23,0,-23};   //前进(Forward)
int8_t car_retreat[4]={0,-23,0,23};   //后退(Backward)
int8_t car_turnLeft[4]={0,20,0,20}; //原地左转(Turn left in place)
int8_t car_stop[4]={0,0,0,0};

void loop()
{
  /* 小车前进(Car moves forward) */
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_forward,4);
  delay(2000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_stop,4);
  delay(1000);
  /* 小车后退(Car moves backward) */
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_retreat,4);
  delay(2000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_stop,4);
  delay(1000);    
  /* 小车原地左转(Car turns left in place) */
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_turnLeft,4);
  delay(600);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_stop,4);
  delay(1000);
  /* 小车向现在的方向前进(Car moves forward in the current direction) */ 
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_forward,4);
  delay(2000);
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR,car_stop,4);
  while(1);
}
