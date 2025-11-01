#include <Wire.h>

// ============ I2C Configuration ============
#define I2C_ADDR        0x34  // Alamat I2C motor driver

// Pin I2C ESP32 (bisa diganti sesuai kebutuhan)
#define SDA_PIN 21  // GPIO21 (default SDA ESP32)
#define SCL_PIN 22  // GPIO22 (default SCL ESP32)

// ============ Register Address ============
#define ADC_BAT_ADDR                  0x00
#define MOTOR_TYPE_ADDR               0x14
#define MOTOR_ENCODER_POLARITY_ADDR   0x15
#define MOTOR_FIXED_PWM_ADDR          0x1F  // PWM control (open-loop)
#define MOTOR_FIXED_SPEED_ADDR        0x33  // Speed control (closed-loop)
#define MOTOR_ENCODER_TOTAL_ADDR      0x3C

// ============ Motor Types ============
#define MOTOR_TYPE_WITHOUT_ENCODER        0
#define MOTOR_TYPE_TT                     1
#define MOTOR_TYPE_N20                    2
#define MOTOR_TYPE_JGB37_520_12V_110RPM   3

// ============ Configuration ============
uint8_t MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM;
uint8_t MotorEncoderPolarity = 0;

// ============ Motor Speed Arrays ============
int8_t forward[4]  = {50, 50, 50, 50};   // Maju
int8_t backward[4] = {-50, -50, -50, -50}; // Mundur
int8_t turnLeft[4] = {-30, -30, 30, 30};  // Belok kiri (contoh)
int8_t turnRight[4] = {30, 30, -30, -30}; // Belok kanan (contoh)
int8_t stop[4]     = {0, 0, 0, 0};        // Stop

// ============ I2C Functions ============

bool WireWriteByte(uint8_t val) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool WireWriteDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for(unsigned int i = 0; i < len; i++) {
    Wire.write(val[i]);
  }
  return (Wire.endTransmission() == 0);
}

bool WireReadDataByte(uint8_t reg, uint8_t &val) {
  if (!WireWriteByte(reg)) return false;
  
  Wire.requestFrom(I2C_ADDR, 1);
  if (Wire.available()) {
    val = Wire.read();
    return true;
  }
  return false;
}

int WireReadDataArray(uint8_t reg, uint8_t *val, unsigned int len) {
  if (!WireWriteByte(reg)) return -1;
  
  Wire.requestFrom(I2C_ADDR, len);
  unsigned int i = 0;
  while (Wire.available() && i < len) {
    val[i++] = Wire.read();
  }
  return i;
}

// ============ Motor Control Functions ============

void setMotorSpeed(int8_t *speeds) {
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)speeds, 4);
}

void setMotorPWM(int8_t *pwm) {
  WireWriteDataArray(MOTOR_FIXED_PWM_ADDR, (uint8_t*)pwm, 4);
}

void stopMotors() {
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)stop, 4);
}

uint16_t readBatteryVoltage() {
  uint8_t data[2];
  WireReadDataArray(ADC_BAT_ADDR, data, 2);
  return data[0] + (data[1] << 8);
}

// ============ Setup ============

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10); // Wait for serial
  
  Serial.println("\n=== ESP32 Motor Driver ===");
  
  // Initialize I2C dengan pin custom (opsional)
  Wire.begin(SDA_PIN, SCL_PIN);
  // Atau pakai default: Wire.begin();
  
  Wire.setClock(100000); // 100kHz I2C speed
  
  delay(200);
  
  // Initialize motor driver
  Serial.println("Initializing motor driver...");
  
  if (!WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1)) {
    Serial.println("ERROR: Failed to set motor type!");
  } else {
    Serial.println("Motor type set successfully");
  }
  delay(10);
  
  if (!WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1)) {
    Serial.println("ERROR: Failed to set encoder polarity!");
  } else {
    Serial.println("Encoder polarity set successfully");
  }
  delay(10);
  
  Serial.println("Motor driver initialized!");
  Serial.println("Ready to control motors\n");
}

// ============ Loop ============

void loop() {
  // Read battery voltage
  uint16_t voltage = readBatteryVoltage();
  Serial.printf("Battery: %d mV\n", voltage);
  
  // Forward
  Serial.println("Moving FORWARD...");
  setMotorSpeed(forward);
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  stopMotors();
  delay(1000);
  
  // Backward
  Serial.println("Moving BACKWARD...");
  setMotorSpeed(backward);
  delay(3000);
  
  // Stop
  Serial.println("STOP");
  stopMotors();
  delay(1000);
  
  // Turn Left
  Serial.println("Turning LEFT...");
  setMotorSpeed(turnLeft);
  delay(2000);
  
  // Stop
  Serial.println("STOP");
  stopMotors();
  delay(1000);
  
  // Turn Right
  Serial.println("Turning RIGHT...");
  setMotorSpeed(turnRight);
  delay(2000);
  
  // Stop
  Serial.println("STOP");
  stopMotors();
  delay(2000);
  
  Serial.println("---");
}
