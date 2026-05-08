/* Copyright (c) 2017 Musa Mahmood
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/** @file
 * @brief Functions for initializing and controlling Texas Instruments ADS1299 analog front-end.
 */

#ifndef ADS1299_H__
#define ADS1299_H__                   
#include "nrf_drv_spi.h"
#include "ble_eeg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#define ADS1299_NUM_REGS              24
#define ADS1299_WRITABLE_REG_COUNT    23
/**
 *	\brief ADS1291_2 register addresses.
 * Consult the ADS1291/2 datasheet and user's guide for more information.
 */
// REG 0x00: ID
#define ADS1299_REGADDR_ID            0x00
// REG 0x01: CONFIG1
#define ADS1299_REGADDR_CONFIG1       0x01
// REG 0x02: CONFIG2
#define ADS1299_REGADDR_CONFIG2       0x02
// REG 0x03: CONFIG3
#define ADS1299_REGADDR_CONFIG3       0x03
// REG 0x04: LOFF
#define ADS1299_REGADDR_LOFF          0x04
// REG 0x05: CH1SET
#define ADS1299_REGADDR_CH1SET        0x05
// REG 0x06: CH2SET
#define ADS1299_REGADDR_CH2SET        0x06
// REG 0x07: CH3SET
#define ADS1299_REGADDR_CH3SET        0x07
// REG 0x08: CH4SET
#define ADS1299_REGADDR_CH4SET        0x08
// REG 0x09: CH5SET
#define ADS1299_REGADDR_CH5SET        0x09
// REG 0x0A: CH6SET
#define ADS1299_REGADDR_CH6SET        0x0A
// REG 0x0B: CH7SET
#define ADS1299_REGADDR_CH7SET        0x0B
// REG 0x0C: CH8SET
#define ADS1299_REGADDR_CH8SET        0x0C
// REG 0x0D: BIAS_SENSP
#define ADS1299_REGADDR_BIAS_SENSP    0x0D
// REG 0x0E: BIAS_SENSN
#define ADS1299_REGADDR_BIAS_SENSN    0x0E
// REG 0x0F: LOFF_SENSP
#define ADS1299_REGADDR_LOFF_SENSP    0x0F
// REG 0x10: LOFF_SENSN
#define ADS1299_REGADDR_LOFF_SENSN    0x10
// REG 0x11: LOFF_FLIP
#define ADS1299_REGADDR_LOFF_FLIP     0x11
// REG 0x12: LOFF_STATP
#define ADS1299_REGADDR_LOFF_STATP    0x12
// REG 0x13: LOFF_STATN
#define ADS1299_REGADDR_LOFF_STATN    0x13
// REG 0x14: GPIO
#define ADS1299_REGADDR_GPIO          0x14
// REG 0x15: MISC1
#define ADS1299_REGADDR_MISC1         0x15
// REG 0x16: MISC2
#define ADS1299_REGADDR_MISC2         0x16
// REG 0x17: CONFIG4
#define ADS1299_REGADDR_CONFIG4       0x17
/**
 *	\brief ADS1299 SPI communication opcodes.
 *	
 * Consult the ADS1299 datasheet and user's guide for more information.
 * For RREG and WREG opcodes, the first byte (opcode) must be ORed with the address of the register to be read/written. 
 * The command is completed with a second byte 000n nnnn, where n nnnn is (# registers to read) - 1.
 */
#define ADS1299_OPC_WAKEUP            0x02    ///< Wake up from standby.
#define ADS1299_OPC_STANDBY           0x04   ///< Enter standby.
#define ADS1299_OPC_RESET             0x06     ///< Reset all registers.
#define ADS1299_OPC_START             0x08     ///< Start data conversions.
#define ADS1299_OPC_STOP              0x0A      ///< Stop data conversions.
#define ADS1299_OPC_OFFSETCAL         0x1A ///< Calibrate channel offset. RESP2.CALIB_ON must be 1. Execute after every PGA gain change.
#define ADS1299_OPC_RDATAC            0x10    ///< Read data continuously (registers cannot be read or written in this mode).
#define ADS1299_OPC_SDATAC            0x11    ///< Stop continuous data read.
#define ADS1299_OPC_RDATA             0x12     ///< Read single data value.
#define ADS1299_OPC_RREG              0x20 ///< Read register value. System must not be in RDATAC mode.
#define ADS1299_OPC_WREG              0x40 ///< Write register value.
/**********************************/

/* ID REGISTER ********************************************************************/

/**
 *  \brief Factory-programmed device ID for ADS1299 & ADS1299-x.
 */
#define ADS1299_4_DEVICE_ID           0x1C //Device ID [0bvvv11100] Where vvv is the version bits
#define ADS1299_6_DEVICE_ID           0x1D //Device ID [0bvvv11101]
#define ADS1299_DEVICE_ID             0x1E   //Device ID [0bvvv11101]
/* DEFAULT REGISTER VALUES ********************************************************/

//Use multi-line copy from excel file.
//0xB6 = 250SPS
//0xB5 = 500SPS
//0xB4 = 1kSPS
//0xB3 = 2kSPS
//0xB2 = 4kSPS
//0xB1 = 8kSPS
//0xB0 = 16kSPS

//0x96 = 250SPS //0x95 = 500SPS //0x94 = 1kSPS //0x93 = 2kSPS //0x92 = 4kSPS //0x91 = 8kSPS //0x90 = 16kSPS
/* CONFIG1 (0x01):
{ [7] = 1 | [6] = DAISY_EN | [5] = CLK_EN | [4:3] = 0b10 | [2:0] = DR } */
#define ADS1299_REGDEFAULT_CONFIG1    0x95 // 95
/* CONFIG2 (0x02):
{ [7:5] = RESERVED (110) | [4] = INT_CAL | [3] = RESERVED | [2] = CAL_AMP | [1:0] = CAL_FREQ |  } */
#define ADS1299_REGDEFAULT_CONFIG2    0xC0
/* NOTE:
To use BIAS_IN only: If you want the BIAS drive to ignore the internal channel summing and only look at 
what's coming into the BIAS_IN pin, you need to set the CONFIG3 register.
Set BIAS_MEAS = 1: This routes the signal from the BIAS_IN pin to the BIAS amplifier.
Crucial Step: If you only want BIAS_IN, you should disable (set to 0) the bits in BIAS_SENSP and BIAS_SENSN. 
This "unplugs" the internal channels from the BIAS amplifier's summing junction.
*/
/* CONFIG3 (0x03):
{ [7] = PD_REFBUF` | [6:5] = RESERVED {11} | [4] = BIAS_MEAS | [3] = BIASREF_INT | [2] = PD_BIAS` | [1] = BIAS_LOFF_SENS | [0] = BIAS_STAT } */
#define ADS1299_REGDEFAULT_CONFIG3    0xEC  // TODO play with and check correctly
/* LOFF (0x04):
{ [7:5] = COMP_TH | [4:2] = ILEAD_OFF | [1:0] = FLEAD_OFF } */
#define ADS1299_REGDEFAULT_LOFF       0x00
/* CH1SET (0x05):
{ [7] = PD1 | [6:4] = GAIN1 | [3] = S RB2 | [2:0] = MUX1 } */
#define ADS1299_REGDEFAULT_CH1SET     0x50
/* CH2SET (0x06):
{ [7] = PD2 | [6:4] = GAIN2 | [3] = SRB2 | [2:0] = MUX2 } */
#define ADS1299_REGDEFAULT_CH2SET     0x50
/* CH3SET (0x07):
{ [7] = PD3 | [6:4] = GAIN3 | [3] = SRB2 | [2:0] = MUX3 } */
#define ADS1299_REGDEFAULT_CH3SET     0x50
/* CH4SET (0x08):
{ [7] = PD4 | [6:4] = GAIN4 | [3] = SRB2 | [2:0] = MUX4 } */
#define ADS1299_REGDEFAULT_CH4SET     0x50
/* CH5SET (0x09):
{ [7] = PD5 | [6:4] = GAIN5 | [3] = SRB2 | [2:0] = MUX5 } */
#define ADS1299_REGDEFAULT_CH5SET     0x50
/* CH6SET (0x0A):
{ [7] = PD6 | [6:4] = GAIN6 | [3] = SRB2 | [2:0] = MUX6 } */
#define ADS1299_REGDEFAULT_CH6SET     0x50
/* CH7SET (0x0B):
{ [7] = PD7 | [6:4] = GAIN7 | [3] = SRB2 | [2:0] = MUX7 } */
#define ADS1299_REGDEFAULT_CH7SET     0x50
/* CH8SET (0x0C):
{ [7] = PD8 | [6:4] = GAIN8 | [3] = SRB2 | [2:0] = MUX8 } */
#define ADS1299_REGDEFAULT_CH8SET     0x50
/* BIAS_SENSP (0x0D):
{ [7:0] = BIASP[8:1] } */
#define ADS1299_REGDEFAULT_BIAS_SENSP 0x00u // 0xFF maybe?
/* BIAS_SENSN (0x0E):
{ [7:0] = BIASN[8:1] } */
#define ADS1299_REGDEFAULT_BIAS_SENSN 0x00
/* LOFF_SENSP (0x0F):
{ [7:0] = LOFFP[8:1] } */
#define ADS1299_REGDEFAULT_LOFF_SENSP 0x00
/* LOFF_SENSN (0x10):
{ [7:0] = LOFFN[8:1] } */
#define ADS1299_REGDEFAULT_LOFF_SENSN 0x00
/* LOFF_FLIP (0x11):
{ [7:0] = LOFF_FLIP[8:1] } */
#define ADS1299_REGDEFAULT_LOFF_FLIP  0x00
/* LOFF_STATP (0x12):
{ [7] = RESERVED | [6] = CLK_DIV | [5] = RESERVED | [4:0] = LOFFP_STAT[5:1] } */
#define ADS1299_REGDEFAULT_LOFF_STATP 0x00
/* LOFF_STATN (0x13):
{ [7] = RESERVED | [6] = CLK_DIV | [5] = RESERVED | [4:0] = LOFFN_STAT[5:1] } */
#define ADS1299_REGDEFAULT_LOFF_STATN 0x00
/* GPIO (0x14):
{ [7:4] = GPIOD[4:1] | [3:0] = GPIOC[4:1] } */
#define ADS1299_REGDEFAULT_GPIO       0x0F
/* MISC1 (0x15):
{ [7:6] = RESERVED | [5] = SRB1 | [4:0] = RESERVED } */
#define ADS1299_REGDEFAULT_MISC1      0x00
/* MISC2 (0x16):
{ [7:0] = RESERVED } */
#define ADS1299_REGDEFAULT_MISC2      0x00
/* CONFIG4 (0x17):
{ [7:0] = RESERVED } */
#define ADS1299_REGDEFAULT_CONFIG4    0x00
/* CONFIG1 bit masks */
#define ADS1299_CONFIG1_CLK_EN_Msk    (1u << 5) /* Enable CLK output on CLK pin */
/* Channel configuration masks */
#define SRB2_MASK                     0x08u  // Mask to set the bit for SRB2 refernce (common)
#define CH_PD_MASK                    0x80u // Mask to set the power down bit for unused channels
/* Common reference for ADS1299 devices */
#define COMMON_REF                    true


//
// 0x00 =  125SPS
// 0x01 =  250SPS
// 0x02 =  500SPS
// 0x03 = 1000SPS
// 0x04 = 2000SPS
// 0x05 = 4000SPS
// 0x06 = 8000SPS
//

/*! @TYPEDEFS: */
typedef int16_t body_voltage_t;

/**************************************************************************************************************************************************
*              Function Prototypes ADS1299-x 																																																			*
**************************************************************************************************************************************************/
void ads1299_spi_init(void);
void ads1299_spi_uninit(void);
void ads1299_spi_init_with_sample_freq(uint8_t spi_sclk);

/* Backward-compatible aliases for legacy call sites. */
#define ads_spi_init                  ads1299_spi_init
#define ads_spi_uninit                ads1299_spi_uninit
void ads1299_start_conv(void);

void ads1299_stop_conv(void);

/**
 *	\brief Initialize the ADS1299-x.
 * This function performs the power-on reset and initialization procedure documented on page 61 of the
 * ADS1299 datasheet, up to "Send SDATAC Command."
 */
void ads1299_powerup_reset(void);

void ads1299_struct_init(void);

void ads1299_init_regs(ble_eeg_t *p_eeg, uint8_t *new_register_values);

void ads1299_init_regs_default(ble_eeg_t *p_eeg);

void ads1299_read_all_registers(ble_eeg_t *p_eeg) ;

void ads1299_powerdn(void);

void ads1299_powerup(void);

void ads1299_standby(ble_eeg_t *p_eeg);

void ads1299_wake(ble_eeg_t *p_eeg);

void ads1299_reset(ble_eeg_t *p_eeg);

void ads1299_soft_start_conversion(ble_eeg_t *p_eeg);

void ads1299_stop_rdatac(ble_eeg_t *p_eeg);

void ads1299_start_rdatac(ble_eeg_t *p_eeg);

void ads1299_check_id(ble_eeg_t *p_eeg);

void get_eeg_voltage_array(ble_eeg_t *p_eeg);
void get_eeg_voltage_array_4ch(ble_eeg_t *p_eeg);
void get_eeg_voltage_array_ch1_2(ble_eeg_t *p_eeg);
void get_eeg_voltage_array_all_ch(ble_eeg_t *p_eeg);

#endif // ADS1299_H__
