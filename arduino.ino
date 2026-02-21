/*
 * ICM-42688-P Minimal Code - Direct Register Access (No Library Required)
 * 
 * This code interfaces with ICM-42688-P using direct I2C register access
 * Based on official ICM-42688-P datasheet (DS-000347 Rev 1.6)
 * 
 * WIRING:
 * =======
 * SDA (pin 14)  -> Arduino A4 (SDA)
 * SCL (pin 13)  -> Arduino A5 (SCL)
 * CS  (pin 12)  -> 3.3V (CRITICAL - enables I2C mode!)
 * SDO/AD0 (1)   -> GND (I2C address 0x68) or 3.3V (address 0x69)
 * FSYNC (pin 9) -> GND
 * VDD (pin 8)   -> 3.3V
 * VDDIO (pin 5) -> 3.3V
 * GND (pin 6)   -> GND
 */

#include <Wire.h>

// I2C address (0x68 if AD0 is GND, 0x69 if AD0 is 3.3V)
#define ICM42688_ADDR 0x68

// Register Bank 0 - commonly used registers (datasheet page 61-88)
#define REG_DEVICE_CONFIG     0x11
#define REG_INT_CONFIG        0x14
#define REG_PWR_MGMT0         0x4E  // Power management
#define REG_GYRO_CONFIG0      0x4F  // Gyro ODR and FS
#define REG_ACCEL_CONFIG0     0x50  // Accel ODR and FS
#define REG_TEMP_DATA1        0x1D  // Temperature high byte
#define REG_TEMP_DATA0        0x1E  // Temperature low byte
#define REG_ACCEL_DATA_X1     0x1F  // Accel X high byte
#define REG_ACCEL_DATA_X0     0x20  // Accel X low byte
#define REG_ACCEL_DATA_Y1     0x21  // Accel Y high byte
#define REG_ACCEL_DATA_Y0     0x22  // Accel Y low byte
#define REG_ACCEL_DATA_Z1     0x23  // Accel Z high byte
#define REG_ACCEL_DATA_Z0     0x24  // Accel Z low byte
#define REG_GYRO_DATA_X1      0x25  // Gyro X high byte
#define REG_GYRO_DATA_X0      0x26  // Gyro X low byte
#define REG_GYRO_DATA_Y1      0x27  // Gyro Y high byte
#define REG_GYRO_DATA_Y0      0x28  // Gyro Y low byte
#define REG_GYRO_DATA_Z1      0x29  // Gyro Z high byte
#define REG_GYRO_DATA_Z0      0x2A  // Gyro Z low byte
#define REG_WHO_AM_I          0x75  // Device ID register
#define REG_REG_BANK_SEL      0x76  // Register bank selection

// Expected WHO_AM_I value for ICM-42688-P
#define ICM42688_WHO_AM_I_VALUE 0x47

// Power Management settings
#define PWR_MGMT0_ACCEL_MODE_LN   0x03  // Accelerometer Low Noise mode
#define PWR_MGMT0_GYRO_MODE_LN    0x0C  // Gyroscope Low Noise mode

// Scale factors (from datasheet Table 1 & 2)
#define ACCEL_SCALE_16G  2048.0f   // LSB/g for ±16g range
#define GYRO_SCALE_2000  16.4f     // LSB/(deg/s) for ±2000 dps range
#define TEMP_SENSITIVITY 132.48f   // LSB/°C
#define TEMP_OFFSET      25.0f     // °C

// Function prototypes
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readRegisters(uint8_t reg, uint8_t count, uint8_t *data);
int16_t combineBytes(uint8_t msb, uint8_t lsb);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  Serial.println("ICM-42688-P Minimal Code (No Library)");
  Serial.println("=====================================\n");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast Mode
  
  delay(100); // Power-up delay
  
  // Check WHO_AM_I register
  Serial.println("Checking device ID...");
  uint8_t whoami = readRegister(REG_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  
  if (whoami != ICM42688_WHO_AM_I_VALUE) {
    Serial.println("❌ Error: Incorrect device ID!");
    Serial.print("Expected: 0x");
    Serial.print(ICM42688_WHO_AM_I_VALUE, HEX);
    Serial.print(", Got: 0x");
    Serial.println(whoami, HEX);
    Serial.println("\nCheck connections:");
    Serial.println("- CS pin MUST be at 3.3V for I2C mode");
    Serial.println("- Verify I2C address (0x68 or 0x69)");
    while (1) {}
  }
  
  Serial.println("✅ Device ID verified!");
  
  // Perform software reset (optional but recommended)
  Serial.println("Performing software reset...");
  writeRegister(REG_DEVICE_CONFIG, 0x01); // Set SOFT_RESET_CONFIG bit
  delay(50); // Wait for reset to complete
  
  // Configure power management
  // Enable both gyro and accel in Low Noise mode (datasheet page 76)
  Serial.println("Configuring power management...");
  writeRegister(REG_PWR_MGMT0, PWR_MGMT0_GYRO_MODE_LN | PWR_MGMT0_ACCEL_MODE_LN);
  delay(50); // Wait for sensors to power up
  
  // Configure accelerometer
  // ODR = 1kHz (0110b), FS = ±16g (000b) - datasheet page 78
  Serial.println("Configuring accelerometer (±16g, 1kHz)...");
  writeRegister(REG_ACCEL_CONFIG0, 0x06);
  
  // Configure gyroscope  
  // ODR = 1kHz (0110b), FS = ±2000dps (000b) - datasheet page 77
  Serial.println("Configuring gyroscope (±2000dps, 1kHz)...");
  writeRegister(REG_GYRO_CONFIG0, 0x06);
  
  delay(100); // Let sensors stabilize
  
  Serial.println("✅ Initialization complete!\n");
  Serial.println("Reading sensor data...\n");
  delay(500);
}

void loop() {
  // Buffer for reading all sensor data at once (14 bytes)
  // ACCEL_DATA (6) + GYRO_DATA (6) + TEMP_DATA (2)
  uint8_t data[14];
  
  // Read all sensor data starting from TEMP_DATA1
  readRegisters(REG_TEMP_DATA1, 14, data);
  
  // Combine bytes to get 16-bit values
  int16_t tempRaw   = combineBytes(data[0], data[1]);
  int16_t accelX    = combineBytes(data[2], data[3]);
  int16_t accelY    = combineBytes(data[4], data[5]);
  int16_t accelZ    = combineBytes(data[6], data[7]);
  int16_t gyroX     = combineBytes(data[8], data[9]);
  int16_t gyroY     = combineBytes(data[10], data[11]);
  int16_t gyroZ     = combineBytes(data[12], data[13]);
  
  // Convert to physical units
  float temperature = (tempRaw / TEMP_SENSITIVITY) + TEMP_OFFSET;
  float ax = accelX / ACCEL_SCALE_16G;  // g
  float ay = accelY / ACCEL_SCALE_16G;  // g
  float az = accelZ / ACCEL_SCALE_16G;  // g
  float gx = gyroX / GYRO_SCALE_2000;   // deg/s
  float gy = gyroY / GYRO_SCALE_2000;   // deg/s
  float gz = gyroZ / GYRO_SCALE_2000;   // deg/s
  
  // Output CSV format: timestamp,ax,ay,az,gx,gy,gz,temp
  Serial.print(millis()); Serial.print(",");
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 2); Serial.print(",");
  Serial.print(gy, 2); Serial.print(",");
  Serial.print(gz, 2); Serial.print(",");
  Serial.println(temperature, 2);
  
  delay(10); // 100 Hz update rate for VBT
}

// ============== I2C Functions ==============

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ICM42688_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(ICM42688_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Don't release bus
  
  Wire.requestFrom(ICM42688_ADDR, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  Wire.beginTransmission(ICM42688_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Don't release bus
  
  Wire.requestFrom(ICM42688_ADDR, count);
  uint8_t i = 0;
  while (Wire.available() && i < count) {
    data[i++] = Wire.read();
  }
}

int16_t combineBytes(uint8_t msb, uint8_t lsb) {
  return (int16_t)((msb << 8) | lsb);
}

/*
 * NOTES:
 * ======
 * 1. This minimal code uses default settings for most features
 * 2. For production use, consider adding:
 *    - Error checking on I2C transactions
 *    - Digital filter configuration
 *    - Interrupt configuration
 *    - FIFO buffer usage
 *    - Calibration routines
 * 
 * 3. Register addresses from datasheet Section 13 (pages 61-107)
 * 4. All configuration values from datasheet specifications
 * 
 * 5. To change ranges:
 *    - Accel: modify REG_ACCEL_CONFIG0 (bits 7:5)
 *      000 = ±16g, 001 = ±8g, 010 = ±4g, 011 = ±2g
 *    - Gyro: modify REG_GYRO_CONFIG0 (bits 7:5)  
 *      000 = ±2000dps, 001 = ±1000dps, etc.
 *    - Update scale factors accordingly
 */