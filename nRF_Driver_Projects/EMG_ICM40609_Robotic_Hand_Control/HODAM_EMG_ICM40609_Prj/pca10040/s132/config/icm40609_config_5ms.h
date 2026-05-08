/* ICM40609 Register Configuration Macros for 5 ms (200 Hz) Sampling. */
#ifndef ICM40609_CONFIG_5MS_H
#define ICM40609_CONFIG_5MS_H

#ifdef __cplusplus
extern "C" {
#endif

/* DEVICE_CONFIG: SOFT_RESET_CONFIG = 1 triggers device reset. */
#define ICM40609_DEVICE_CONFIG_SOFT_RESET         0x01u

/* PWR_MGMT0: set both accel and gyro to Low Noise mode (and keep TEMP enabled). */
#define ICM40609_PWR_MGMT0_ACCEL_GYRO_ON          0x0Fu

/* Sampling configuration (5 ms interval = 200 Hz)
 *
 * ACCEL_CONFIG0:
 *   ACCEL_FS_SEL = 0b011 (±4 g)
 *   ACCEL_ODR    = 0b0111 (200 Hz)
 *
 * GYRO_CONFIG0:
 *   GYRO_FS_SEL = 0b001 (±1000 dps)
 *   GYRO_ODR    = 0b0111 (200 Hz)
 */
#define ICM40609_ACCEL_CONFIG0_VAL_200HZ_4G       0x67u
#define ICM40609_GYRO_CONFIG0_VAL_200HZ_1000DPS   0x27u

/* Other macros */
#define ICM40609_WHO_AM_I_VAL                     0x3Bu

#ifdef __cplusplus
}
#endif

#endif /* ICM40609_CONFIG_5MS_H */

