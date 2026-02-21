// =============================================================================
// ICM-42688-P Register Definitions
// Based on DS-000347 Rev 1.2
// =============================================================================

// ---- SPI Protocol ----
// Read:  MSB=1, [6:0]=addr, then 8-bit data
// Write: MSB=0, [6:0]=addr, then 8-bit data
// SPI Mode 0 or Mode 3 (CPOL=0/CPHA=0 or CPOL=1/CPHA=1)

// ---- Bank 0 Registers ----
`define REG_DEVICE_CONFIG   8'h11   // [0] SOFT_RESET_CONFIG
`define REG_DRIVE_CONFIG    8'h13
`define REG_INT_CONFIG      8'h14
`define REG_FIFO_CONFIG     8'h16
`define REG_TEMP_DATA1      8'h1D   // [7:0] TEMP_DATA[15:8]
`define REG_TEMP_DATA0      8'h1E   // [7:0] TEMP_DATA[7:0]
`define REG_ACCEL_DATA_X1   8'h1F
`define REG_ACCEL_DATA_X0   8'h20
`define REG_ACCEL_DATA_Y1   8'h21
`define REG_ACCEL_DATA_Y0   8'h22
`define REG_ACCEL_DATA_Z1   8'h23
`define REG_ACCEL_DATA_Z0   8'h24
`define REG_GYRO_DATA_X1    8'h25
`define REG_GYRO_DATA_X0    8'h26
`define REG_GYRO_DATA_Y1    8'h27
`define REG_GYRO_DATA_Y0    8'h28
`define REG_GYRO_DATA_Z1    8'h29
`define REG_GYRO_DATA_Z0    8'h2A
`define REG_INT_STATUS      8'h2D
`define REG_FIFO_COUNTH     8'h2E
`define REG_FIFO_COUNTL     8'h2F
`define REG_FIFO_DATA       8'h30
`define REG_INT_STATUS2     8'h37
`define REG_INT_STATUS3     8'h38
`define REG_SIGNAL_PATH_RST 8'h4B
`define REG_INTF_CONFIG0    8'h4C
`define REG_INTF_CONFIG1    8'h4D
`define REG_PWR_MGMT0       8'h4E
`define REG_GYRO_CONFIG0    8'h4F
`define REG_ACCEL_CONFIG0   8'h50
`define REG_GYRO_CONFIG1    8'h51
`define REG_GYRO_ACCEL_CFG0 8'h52
`define REG_ACCEL_CONFIG1   8'h53
`define REG_TMST_CONFIG     8'h54
`define REG_FIFO_CONFIG1    8'h5F
`define REG_FIFO_CONFIG2    8'h60
`define REG_FIFO_CONFIG3    8'h61
`define REG_INT_CONFIG0     8'h63
`define REG_INT_CONFIG1     8'h64
`define REG_INT_SOURCE0     8'h65
`define REG_WHO_AM_I        8'h75   // Expected: 0x47
`define REG_BANK_SEL        8'h76

// ---- WHO_AM_I expected value ----
`define ICM42688_WHO_AM_I   8'h47

// ---- PWR_MGMT0 bits ----
// [3:2] GYRO_MODE: 00=off, 01=standby, 11=low-noise
// [1:0] ACCEL_MODE: 00=off, 01=off, 10=low-power, 11=low-noise
`define GYRO_MODE_OFF       2'b00
`define GYRO_MODE_STANDBY   2'b01
`define GYRO_MODE_LN        2'b11
`define ACCEL_MODE_OFF      2'b00
`define ACCEL_MODE_LP       2'b10
`define ACCEL_MODE_LN       2'b11

// ---- GYRO_CONFIG0 ----
// [7:5] FS_SEL: 000=±2000dps, 001=±1000, 010=±500, 011=±250, 100=±125
// [3:0] ODR:   0110=1kHz LN, 0101=2kHz, 0100=4kHz, 0011=8kHz
`define GYRO_FS_2000        3'b000
`define GYRO_FS_1000        3'b001
`define GYRO_FS_500         3'b010
`define GYRO_FS_250         3'b011
`define GYRO_ODR_1KHZ       4'b0110
`define GYRO_ODR_2KHZ       4'b0101
`define GYRO_ODR_4KHZ       4'b0100

// ---- ACCEL_CONFIG0 ----
// [7:5] FS_SEL: 000=±16g, 001=±8g, 010=±4g, 011=±2g
// [3:0] ODR:   same as gyro
`define ACCEL_FS_16G        3'b000
`define ACCEL_FS_8G         3'b001
`define ACCEL_FS_4G         3'b010
`define ACCEL_FS_2G         3'b011
`define ACCEL_ODR_1KHZ      4'b0110

// ---- INTF_CONFIG0 ----
// [1:0] UI_SIFS_CFG: 00=SPI+I2C, 01=SPI+I3C, 10=disable I2C, 11=disable I2C
`define INTF_DIS_I2C        2'b11

// ---- INT_STATUS bits ----
`define INT_STATUS_DATA_RDY 1'b1    // bit 3

// ---- SPI read/write bit ----
`define SPI_READ            1'b1    // bit 7 of first byte
`define SPI_WRITE           1'b0
