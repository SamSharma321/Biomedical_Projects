/* ICM20649 Register Configuration Macros for 5 ms (200 Hz) Sampling with Interrupts */
#ifndef ICM20649_CONFIG_5MS_H
#define ICM20649_CONFIG_5MS_H             
#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * SAMPLING RATE CONFIGURATION (5 ms interval = 200 Hz)
 * ============================================================================
 * 
 * The ICM20649 uses the following formula for sample rate:
 *   Sample_Rate = 1100 / (1 + SMPLRT_DIV)  [Hz]
 * For 200 Hz (5 ms):
 *   200 = 1100 / (1 + SMPLRT_DIV)
 *   SMPLRT_DIV = 4
 */

/* Accelerometer Sample Rate Divisor (Bank 1, Reg 0x10-0x11)
 * ACCEL_SMPLRT_DIV = 4 gives 200 Hz (5 ms interval)
 * Register uses 2 bytes: [0x10] = MSB, [0x11] = LSB
 * Value: 0x0004 = 4 decimal
 */
#define ICM20649_ACCEL_SMPLRT_DIV_MSB_5MS 0x00u  /* Accel SMPLRT_DIV[15:8] */
#define ICM20649_ACCEL_SMPLRT_DIV_LSB_5MS 0x04u  /* Accel SMPLRT_DIV[7:0]  */
/* Gyroscope Sample Rate Divisor (Bank 1, Reg 0x00)
 * GYRO_SMPLRT_DIV = 4 gives 200 Hz (5 ms interval)
 * Note: Gyro sample rate = 1100 / (1 + GYRO_SMPLRT_DIV) when gyro is in normal mode
 */
#define ICM20649_GYRO_SMPLRT_DIV_5MS      0x04u  /* Gyro SMPLRT_DIV = 4 */
/* ============================================================================
 * POWER MANAGEMENT CONFIGURATION
 * ============================================================================ */

/* PWR_MGMT_1 (Bank 0, Reg 0x06)
 * Bits: DEVICE_RESET | SLEEP | LP_EN | CLK_SEL[2:0]
 * For normal operation with all sensors enabled:
 *   Bit 7: DEVICE_RESET = 0 (no reset)
 *   Bit 6: SLEEP = 0 (not in sleep mode)
 *   Bit 5: LP_EN = 0 (low power mode disabled for accurate timing)
 *   Bits 2:0: CLK_SEL = 001 (use PLL with X gyro reference clock)
 */
#define ICM20649_PWR_MGMT_1_NORMAL_OP     0x01u  /* 0b00000001 */
/* PWR_MGMT_2 (Bank 0, Reg 0x07)
 * Bits: ACCEL_LP_CLK_SEL | RESERVED | ZG_DIS | YG_DIS | XG_DIS | ZA_DIS | YA_DIS | XA_DIS
 * For normal operation with all sensors enabled:
 *   0x00u = all sensors enabled (no disables)
 */
#define ICM20649_PWR_MGMT_2_NORMAL_OP     0x00u  /* 0b00000000 */
/* LP_CONFIG (Bank 0, Reg 0x05)
 * Bits: I2C_MST_CYCLE | ACCEL_CYCLE | GYRO_CYCLE | Others
 * For normal continuous operation (interrupt-driven):
 *   0x00u = disable low power cycles, continuous operation
 */
#define ICM20649_LP_CONFIG_NORMAL_OP      0x00u  /* 0b00000000 */
/* ============================================================================
 * INTERRUPT CONFIGURATION
 * ============================================================================
 * 
 * To generate an interrupt every 5 ms with data ready:
 * 1. Enable Data Ready Interrupt (INT_ENABLE)
 * 2. Configure INT_PIN_CFG for proper edge detection and latch
 */

/* INT_PIN_CFG (Bank 0, Reg 0x0F) */
#define ICM20649_INT_PIN_CFG_DATA_RDY     0x30u  /* 0b00110000 - Latch enabled */
#define ICM20649_INT_ENABLE_DATA_RDY      0x08u  /* 0b00001000 */ // TODO required INT on wakeup?
#define ICM20649_INT_ENABLE_1_DATA_RDY    0x01u  /* 0b00000001 */
#define ICM20649_INT_ENABLE_2_DATA_RDY    0x00u  /* 0b00000001 */
/* ============================================================================
 * ACCELEROMETER CONFIGURATION
 * ============================================================================
 */
/* Full scale range: ±4g */
#define ICM20649_ACCEL_CONFIG_1_VAL       0x1Bu /* 4G with DLPF enabled */
#define ICM20649_ACCEL_CONFIG_2_VAL       0x00u  /* 0b00000000 */
/* ============================================================================
 * GYROSCOPE CONFIGURATION
 * ============================================================================
 */
/* Full scale range: 1000 dps + GYRO filter settings */
#define ICM20649_GYRO_CONFIG_1_VAL        0x1Bu  /* 0b00000000 */
#define ICM20649_GYRO_CONFIG_2_DEFAULT    0x00u  /* 0b00000000 */
/*
 * ============================================================================
 * INTERRUPT HANDLING
 * ============================================================================
 * In your interrupt handler or timer callback:
 * 1. Read INT_STATUS to confirm data ready interrupt:
 *    uint8_t int_status;
 *    icm20649_read_reg(&m_icm20649[0], ICM20649_REG_INT_STATUS, &int_status);
 *    if (int_status & ICM20649_INT_STATUS_DATA_RDY_MASK) {
 *        // Read sensor data via TWI
 *        icm20649_read_sample(&m_icm20649[0], raw_sample_14b, &icm_sample);
 *    }
 * 2. The interrupt is cleared by:
 *    - Reading INT_STATUS register (as shown above), OR
 *    - Reading any accelerometer or gyroscope data register
 */

/* OTHER MACROS */
#define ICM20649_WHO_AM_I_VAL             0xE1u
#define ICM20649_USER_CTRL_VAL            0x00u // 0b'00000000
#define PWR_MGMT_1_NORMAL_OP_VAL          0x09u // 0b'00001001
#define PWR_MGMT_1_DEVICE_RESET_VAL       0x80u
#define PWR_MGMT_1_DEVICE_SLEEP_VAL       0x40u
#ifdef __cplusplus
}
#endif

#endif /* ICM20649_CONFIG_5MS_H */
