#include <Wire.h>
#include <Arduino.h>

// ==== KONFIG I2C ESP32 S3 Uno====
// Sesuaikan dengan wiring Anda:
// Contoh pakai bus utama: SDA=8, SCL=9
// Jika perlu bus terpisah (mis. IMU/OLED di 18/19), pindahkan modul motor ke 21/22.
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_FREQ 100000  // 100 kHz untuk stabilitas

#define I2C_ADDR        0x34

#define ADC_BAT_ADDR                  0x00
#define MOTOR_TYPE_ADDR               0x14
#define MOTOR_ENCODER_POLARITY_ADDR   0x15
#define MOTOR_FIXED_PWM_ADDR          0x1F
#define MOTOR_FIXED_SPEED_ADDR        0x33
#define MOTOR_ENCODER_TOTAL_ADDR      0x3C

#define MOTOR_TYPE_WITHOUT_ENCODER        0
#define MOTOR_TYPE_TT                     1
#define MOTOR_TYPE_N20                    2
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3

uint8_t dataBuf[20];
int8_t  p1[4] = {50, 50, 50, 50};
int8_t  p2[4] = {-20, -20, -20, -20};
int8_t  stopv[4] = {0, 0, 0, 0};
uint8_t EncodeReset[16] = {0};
int32_t EncodeTotal[4];

static bool writeByte(uint8_t val) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool writeArray(uint8_t reg, const uint8_t* val, size_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.write(val, len);
  return (Wire.endTransmission() == 0);
}

static int readArray(uint8_t reg, uint8_t* val, size_t len) {
  if (!writeByte(reg)) return -1;
  //delayMicroseconds(300); // beri waktu perangkat menyiapkan data
  size_t n = Wire.requestFrom((int)I2C_ADDR, (int)len);
  
  if (n != len) return -1;
  for (size_t i=0; i<len; ++i) val[i] = Wire.read();
  return (int)len;
}

static bool readByte(uint8_t reg, uint8_t &val) {
  int r = readArray(reg, &val, 1);
  return (r == 1);
}

uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
uint8_t MotorEncoderPolarity = 0;

void setup() {
  Serial.begin(115200);
  delay(50);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);

  delay(200);
  writeArray(MOTOR_TYPE_ADDR, &MotorType, 1);
  delay(5);
  writeArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1);
}

static int16_t to_u16le(const uint8_t* b) {
  return (int16_t)(b[0] | (b[1] << 8));
}

static int32_t to_i32le(const uint8_t* b) {
  return (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
}

void loop() {
  // Baca tegangan
  if (readArray(ADC_BAT_ADDR, dataBuf, 2) == 2) {
    uint16_t v = (uint16_t)(dataBuf[0] | (dataBuf[1] << 8));
    Serial.printf("V = %umV\n", v);
  } else {
    Serial.println("Baca ADC gagal");
  }

  // Maju
  writeArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)p1, 4);
  delay(3000);

  // Mundur
  writeArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)p2, 4);
  delay(3000);

  // Baca total pulsa (4 x int32 LE)
  if (readArray(MOTOR_ENCODER_TOTAL_ADDR, dataBuf, 16) == 16) {
    for (int i=0;i<4;i++) {
      EncodeTotal[i] = to_i32le(&dataBuf[i*4]);
    }
    Serial.printf("Encode1=%ld Encode2=%ld Encode3=%ld Encode4=%ld\n",
                  (long)EncodeTotal[0], (long)EncodeTotal[1],
                  (long)EncodeTotal[2], (long)EncodeTotal[3]);
  } else {
    Serial.println("Baca encoder gagal");
  }

  // Stop
  writeArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)stopv, 4);
  delay(2000);

  // Reset counter
  writeArray(MOTOR_ENCODER_TOTAL_ADDR, EncodeReset, 16);

  if (readArray(MOTOR_ENCODER_TOTAL_ADDR, dataBuf, 16) == 16) {
    for (int i=0;i<4;i++) EncodeTotal[i] = to_i32le(&dataBuf[i*4]);
    Serial.printf("Reset -> E1=%ld E2=%ld E3=%ld E4=%ld\n",
                  (long)EncodeTotal[0], (long)EncodeTotal[1],
                  (long)EncodeTotal[2], (long)EncodeTotal[3]);
  }
  delay(1000);
}
