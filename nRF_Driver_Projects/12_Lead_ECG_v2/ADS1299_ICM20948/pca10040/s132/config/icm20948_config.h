/* ICM20948 Register Configuration Macros for 5 ms (200 Hz) Sampling with Interrupts */
#ifndef ICM20948_CONFIG_5MS_H
#define ICM20948_CONFIG_5MS_H

#ifdef __cplusplus
extern "C" {
#endif
/* ============================================================================
 * Default configuration (mirrors the ICM20948 5ms/200Hz setup).
 * ============================================================================
 */
/* Bank 0 */
#define ICM20948_USER_CTRL_VAL 0x00u
#define ICM20948_PWR_MGMT_1_NORMAL_OP 0x01u
#define ICM20948_PWR_MGMT_2_NORMAL_OP 0x00u
#define ICM20948_LP_CONFIG_NORMAL_OP 0x00u
#define ICM20948_INT_PIN_CFG_DATA_RDY 0x30u
#define ICM20948_INT_ENABLE_DATA_RDY 0x00u
#define ICM20948_INT_ENABLE_1_DATA_RDY 0x01u

/* Bank 2 */
#define ICM20948_GYRO_SMPLRT_DIV_5MS 0x04u
#define ICM20948_ACCEL_SMPLRT_DIV_MSB_5MS 0x00u
#define ICM20948_ACCEL_SMPLRT_DIV_LSB_5MS 0x04u
#define ICM20948_ACCEL_CONFIG_1_VAL 0x1Bu
#define ICM20948_ACCEL_CONFIG_2_VAL 0x00u
#define ICM20948_GYRO_CONFIG_1_VAL 0x1Bu
#define ICM20948_GYRO_CONFIG_2_DEFAULT 0x00u

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_CONFIG_5MS_H */
