#include <Wire.h>

// ============ I2C Configuration ============
#define I2C_ADDR 0x34

// Pin I2C - Auto-detect board
#if defined(ESP32)
  // ESP32-S3 Uno Clone: IO5=SDA, IO4=SCL (sesuai posisi A4/A5 di UNO)
  #define SDA_PIN 8   // ← Ganti dari IO5 ke IO8
  #define SCL_PIN 9   // ← Ganti dari IO4 ke IO9
#endif

// ============ Register Address ============
#define ADC_BAT_ADDR                  0x00
#define MOTOR_TYPE_ADDR               0x14
#define MOTOR_ENCODER_POLARITY_ADDR   0x15
#define MOTOR_FIXED_PWM_ADDR          0x1F
#define MOTOR_FIXED_SPEED_ADDR        0x33
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
int8_t forward[4]     = {50, 50, 50, 50};      // Maju
int8_t backward[4]    = {-50, -50, -50, -50};  // Mundur
int8_t turnLeft[4]    = {-30, -30, 30, 30};    // Belok kiri
int8_t turnRight[4]   = {30, 30, -30, -30};    // Belok kanan
int8_t rotateLeft[4]  = {-40, -40, 40, 40};    // Rotasi kiri (di tempat)
int8_t rotateRight[4] = {40, 40, -40, -40};    // Rotasi kanan (di tempat)
int8_t stop_motors[4] = {0, 0, 0, 0};          // Stop

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
  WireWriteDataArray(MOTOR_FIXED_SPEED_ADDR, (uint8_t*)stop_motors, 4);
}

uint16_t readBatteryVoltage() {
  uint8_t data[2];
  WireReadDataArray(ADC_BAT_ADDR, data, 2);
  return data[0] + (data[1] << 8);
}

// ============ I2C Scanner (Debug) ============

void scanI2C() {
  Serial.println("\n=== I2C Scanner ===");
  byte count = 0;
  
  for(byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  
  if (count == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(count);
    Serial.println(" device(s)");
  }
  Serial.println();
}

// ============ Setup ============

void setup() {
  // Serial initialization
  #if defined(ESP32)
    Serial.begin(115200);
    delay(1000);  // ESP32 needs time to init serial
    Serial.println("\n\n=== ESP32-S3 Motor Driver ===");
  #else
    Serial.begin(9600);
    Serial.println("\n=== Arduino UNO Motor Driver ===");
  #endif
  
  // I2C initialization - CONDITIONAL
  #if defined(ESP32)
    Wire.begin(SDA_PIN, SCL_PIN);  // ESP32-S3: IO5=SDA, IO4=SCL
    Wire.setClock(100000);         // 100kHz I2C speed
    Serial.print("I2C Pins: SDA=GPIO");
    Serial.print(SDA_PIN);
    Serial.print(", SCL=GPIO");
    Serial.println(SCL_PIN);
  #else
    Wire.begin();  // Arduino UNO: A4=SDA, A5=SCL
    Serial.println("I2C Pins: A4=SDA, A5=SCL");
  #endif
  
  delay(200);
  
  // Scan I2C bus
  scanI2C();
  
  // Motor driver initialization
  Serial.println("Initializing motor driver...");
  
  bool initSuccess = true;
  
  // Set motor type
  if (!WireWriteDataArray(MOTOR_TYPE_ADDR, &MotorType, 1)) {
    Serial.println("ERROR: Failed to set motor type!");
    initSuccess = false;
  } else {
    Serial.print("Motor type set: ");
    Serial.println(MotorType);
  }
  delay(10);
  
  // Set encoder polarity
  if (!WireWriteDataArray(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1)) {
    Serial.println("ERROR: Failed to set encoder polarity!");
    initSuccess = false;
  } else {
    Serial.print("Encoder polarity: ");
    Serial.println(MotorEncoderPolarity);
  }
  delay(10);
  
  if (initSuccess) {
    Serial.println("\n*** Motor driver ready! ***\n");
  } else {
    Serial.println("\n!!! Initialization FAILED !!!");
    Serial.println("Check wiring and I2C connections\n");
  }
  
  delay(1000);
}

// ============ Loop ============

void loop() {
  // Read battery voltage
  uint16_t voltage = readBatteryVoltage();
  Serial.print("Battery: ");
  Serial.print(voltage);
  Serial.println(" mV");
  
  // === Test Sequence ===
  
  // 1. Forward
  Serial.println(">>> FORWARD");
  setMotorSpeed(forward);
  delay(3000);
  
  Serial.println(">>> STOP");
  stopMotors();
  delay(1000);
  
  // 2. Backward
  Serial.println(">>> BACKWARD");
  setMotorSpeed(backward);
  delay(3000);
  
  Serial.println(">>> STOP");
  stopMotors();
  delay(1000);
  
  // 3. Turn Left
  Serial.println(">>> TURN LEFT");
  setMotorSpeed(turnLeft);
  delay(2000);
  
  Serial.println(">>> STOP");
  stopMotors();
  delay(1000);
  
  // 4. Turn Right
  Serial.println(">>> TURN RIGHT");
  setMotorSpeed(turnRight);
  delay(2000);
  
  Serial.println(">>> STOP");
  stopMotors();
  delay(1000);
  
  // 5. Rotate Left (in place)
  Serial.println(">>> ROTATE LEFT");
  setMotorSpeed(rotateLeft);
  delay(2000);
  
  Serial.println(">>> STOP");
  stopMotors();
  delay(1000);
  
  // 6. Rotate Right (in place)
  Serial.println(">>> ROTATE RIGHT");
  setMotorSpeed(rotateRight);
  delay(2000);
  
  Serial.println(">>> STOP");
  stopMotors();
  delay(2000);
  
  Serial.println("--------------------------------------\n");
}