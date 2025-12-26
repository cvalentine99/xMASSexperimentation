/**
 * @file lmk05318b_regs.h
 * @brief Texas Instruments LMK05318B Register Definitions
 * 
 * Register map for the LMK05318B Ultra-Low Jitter Network Synchronizer
 * Based on TI LMK05318B Datasheet (SNAS762) and Programmer's Guide (SNAU254E)
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#ifndef LMK05318B_REGS_H
#define LMK05318B_REGS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Device Identification Registers (Page 0)
 *===========================================================================*/

#define LMK05318B_REG_VENDOR_ID_LSB         0x0000  /* Vendor ID LSB (0x51) */
#define LMK05318B_REG_VENDOR_ID_MSB         0x0001  /* Vendor ID MSB (0x04) */
#define LMK05318B_REG_PRODUCT_ID_LSB        0x0002  /* Product ID LSB (0x0B) */
#define LMK05318B_REG_PRODUCT_ID_MSB        0x0003  /* Product ID MSB (0x35) */
#define LMK05318B_REG_PART_ID_0             0x0004  /* Part ID byte 0 */
#define LMK05318B_REG_PART_ID_1             0x0005  /* Part ID byte 1 */
#define LMK05318B_REG_PART_ID_2             0x0006  /* Part ID byte 2 */
#define LMK05318B_REG_PART_ID_3             0x0007  /* Part ID byte 3 */

/* Expected Device ID values */
#define LMK05318B_VENDOR_ID                 0x0451  /* Texas Instruments */
#define LMK05318B_PRODUCT_ID                0x350B  /* LMK05318B */

/*============================================================================
 * Device Control Registers
 *===========================================================================*/

#define LMK05318B_REG_DEV_CTL               0x0008  /* Device Control */
#define LMK05318B_REG_DEV_CTL_RESERVED      0x0009  /* Reserved */
#define LMK05318B_REG_I2C_ADDR_GPIO1_SW     0x000A  /* I2C Address / GPIO1 SW */
#define LMK05318B_REG_EEPROM_REV            0x000B  /* EEPROM Revision */
#define LMK05318B_REG_RESET_SW              0x000C  /* Reset and SYNC Control */

/* Reset and SYNC Control Register (0x0C) Bit Definitions */
#define LMK05318B_RESET_SW_SOFT_RST         (1 << 7)  /* Software Reset */
#define LMK05318B_RESET_SW_SYNC_SW          (1 << 6)  /* Software SYNC trigger */
#define LMK05318B_RESET_SW_RESERVED_MASK    0x3F      /* Reserved bits */

/*============================================================================
 * Status Registers
 *===========================================================================*/

#define LMK05318B_REG_STATUS_0              0x000D  /* Status Register 0 */
#define LMK05318B_REG_STATUS_1              0x000E  /* Status Register 1 */
#define LMK05318B_REG_STATUS_2              0x000F  /* Status Register 2 */
#define LMK05318B_REG_STATUS_3              0x0010  /* Status Register 3 */

/* Status Register 0 (0x0D) Bit Definitions */
#define LMK05318B_STATUS_PLL1_LOCK          (1 << 0)  /* PLL1 Lock Status */
#define LMK05318B_STATUS_PLL2_LOCK          (1 << 1)  /* PLL2 Lock Status */
#define LMK05318B_STATUS_DPLL_LOCK          (1 << 2)  /* DPLL Lock Status */
#define LMK05318B_STATUS_APLL1_LOCK         (1 << 3)  /* APLL1 Lock Status */
#define LMK05318B_STATUS_APLL2_LOCK         (1 << 4)  /* APLL2 Lock Status */

/* Status Register 1 (0x0E) Bit Definitions */
#define LMK05318B_STATUS_LOPL               (1 << 0)  /* Loss of Phase Lock */
#define LMK05318B_STATUS_LOFL               (1 << 1)  /* Loss of Frequency Lock */
#define LMK05318B_STATUS_LOL                (1 << 2)  /* Loss of Lock */

/*============================================================================
 * Input Reference Registers
 *===========================================================================*/

#define LMK05318B_REG_INPUT_SEL             0x0020  /* Input Selection */
#define LMK05318B_REG_PRIREF_DIV_LSB        0x0021  /* Primary Reference Divider LSB */
#define LMK05318B_REG_PRIREF_DIV_MSB        0x0022  /* Primary Reference Divider MSB */
#define LMK05318B_REG_SECREF_DIV_LSB        0x0023  /* Secondary Reference Divider LSB */
#define LMK05318B_REG_SECREF_DIV_MSB        0x0024  /* Secondary Reference Divider MSB */

/* Input Selection Register (0x20) Bit Definitions */
#define LMK05318B_INPUT_SEL_PRIREF          0x00    /* Select Primary Reference */
#define LMK05318B_INPUT_SEL_SECREF          0x01    /* Select Secondary Reference */
#define LMK05318B_INPUT_SEL_AUTO            0x02    /* Automatic Switchover */

/*============================================================================
 * DPLL Configuration Registers
 *===========================================================================*/

#define LMK05318B_REG_DPLL_CTRL             0x0030  /* DPLL Control */
#define LMK05318B_REG_DPLL_N_DIV_0          0x0031  /* DPLL N Divider byte 0 */
#define LMK05318B_REG_DPLL_N_DIV_1          0x0032  /* DPLL N Divider byte 1 */
#define LMK05318B_REG_DPLL_N_DIV_2          0x0033  /* DPLL N Divider byte 2 */
#define LMK05318B_REG_DPLL_N_DIV_3          0x0034  /* DPLL N Divider byte 3 */
#define LMK05318B_REG_DPLL_N_DIV_4          0x0035  /* DPLL N Divider byte 4 */
#define LMK05318B_REG_DPLL_FRAC_NUM_0       0x0036  /* DPLL Fractional Numerator byte 0 */
#define LMK05318B_REG_DPLL_FRAC_NUM_1       0x0037  /* DPLL Fractional Numerator byte 1 */
#define LMK05318B_REG_DPLL_FRAC_NUM_2       0x0038  /* DPLL Fractional Numerator byte 2 */
#define LMK05318B_REG_DPLL_FRAC_NUM_3       0x0039  /* DPLL Fractional Numerator byte 3 */
#define LMK05318B_REG_DPLL_FRAC_DEN_0       0x003A  /* DPLL Fractional Denominator byte 0 */
#define LMK05318B_REG_DPLL_FRAC_DEN_1       0x003B  /* DPLL Fractional Denominator byte 1 */
#define LMK05318B_REG_DPLL_FRAC_DEN_2       0x003C  /* DPLL Fractional Denominator byte 2 */
#define LMK05318B_REG_DPLL_FRAC_DEN_3       0x003D  /* DPLL Fractional Denominator byte 3 */

/*============================================================================
 * PLL Configuration Registers
 *===========================================================================*/

#define LMK05318B_REG_PLL1_CTRL             0x0040  /* PLL1 Control */
#define LMK05318B_REG_PLL1_N_DIV_LSB        0x0041  /* PLL1 N Divider LSB */
#define LMK05318B_REG_PLL1_N_DIV_MSB        0x0042  /* PLL1 N Divider MSB */
#define LMK05318B_REG_PLL2_CTRL             0x0043  /* PLL2 Control */
#define LMK05318B_REG_PLL2_N_DIV_LSB        0x0044  /* PLL2 N Divider LSB */
#define LMK05318B_REG_PLL2_N_DIV_MSB        0x0045  /* PLL2 N Divider MSB */

/*============================================================================
 * SYNC Control Registers
 *===========================================================================*/

#define LMK05318B_REG_PLL_SYNC_EN           0x0046  /* PLL SYNC Enable */
#define LMK05318B_REG_CH_SYNC_EN            0x0047  /* Channel SYNC Enable */

/* PLL SYNC Enable Register (0x46) Bit Definitions */
#define LMK05318B_PLL_SYNC_PLL1_P1_EN       (1 << 0)  /* PLL1 P1 SYNC Enable */
#define LMK05318B_PLL_SYNC_PLL2_P1_EN       (1 << 1)  /* PLL2 P1 SYNC Enable */
#define LMK05318B_PLL_SYNC_PLL2_P2_EN       (1 << 2)  /* PLL2 P2 SYNC Enable */

/* Channel SYNC Enable Register (0x47) Bit Definitions */
#define LMK05318B_CH_SYNC_CH0_1_EN          (1 << 0)  /* CH0/CH1 SYNC Enable */
#define LMK05318B_CH_SYNC_CH2_3_EN          (1 << 1)  /* CH2/CH3 SYNC Enable */
#define LMK05318B_CH_SYNC_CH4_EN            (1 << 2)  /* CH4 SYNC Enable */
#define LMK05318B_CH_SYNC_CH5_EN            (1 << 3)  /* CH5 SYNC Enable */
#define LMK05318B_CH_SYNC_CH6_EN            (1 << 4)  /* CH6 SYNC Enable */
#define LMK05318B_CH_SYNC_CH7_EN            (1 << 5)  /* CH7 SYNC Enable */
#define LMK05318B_CH_SYNC_ALL               0x3F      /* All channels SYNC Enable */

/*============================================================================
 * Output Channel Registers (CH0-CH7)
 *===========================================================================*/

/* Base addresses for each output channel */
#define LMK05318B_REG_CH0_BASE              0x0050
#define LMK05318B_REG_CH1_BASE              0x0058
#define LMK05318B_REG_CH2_BASE              0x0060
#define LMK05318B_REG_CH3_BASE              0x0068
#define LMK05318B_REG_CH4_BASE              0x0070
#define LMK05318B_REG_CH5_BASE              0x0078
#define LMK05318B_REG_CH6_BASE              0x0080
#define LMK05318B_REG_CH7_BASE              0x0088

/* Channel register offsets (add to base address) */
#define LMK05318B_CH_CTRL_OFFSET            0x00    /* Channel Control */
#define LMK05318B_CH_DIV_LSB_OFFSET         0x01    /* Channel Divider LSB */
#define LMK05318B_CH_DIV_MSB_OFFSET         0x02    /* Channel Divider MSB */
#define LMK05318B_CH_FORMAT_OFFSET          0x03    /* Output Format */
#define LMK05318B_CH_DELAY_LSB_OFFSET       0x04    /* Delay LSB */
#define LMK05318B_CH_DELAY_MSB_OFFSET       0x05    /* Delay MSB */
#define LMK05318B_CH_PHASE_OFFSET           0x06    /* Phase Adjust */
#define LMK05318B_CH_RESERVED_OFFSET        0x07    /* Reserved */

/* Channel Control Bit Definitions */
#define LMK05318B_CH_CTRL_EN                (1 << 0)  /* Channel Enable */
#define LMK05318B_CH_CTRL_MUTE              (1 << 1)  /* Channel Mute */
#define LMK05318B_CH_CTRL_PD                (1 << 2)  /* Power Down */

/* Output Format Values */
#define LMK05318B_OUT_FMT_AC_LVPECL         0x00    /* AC-LVPECL */
#define LMK05318B_OUT_FMT_AC_CML            0x01    /* AC-CML */
#define LMK05318B_OUT_FMT_AC_LVDS           0x02    /* AC-LVDS */
#define LMK05318B_OUT_FMT_HSCL              0x03    /* HSCL */
#define LMK05318B_OUT_FMT_LVCMOS_1V8        0x04    /* 1.8V LVCMOS */

/*============================================================================
 * SYSREF / 1PPS Sync Registers
 *===========================================================================*/

#define LMK05318B_REG_SYSREF_CTRL           0x00FC  /* SYSREF Control */
#define LMK05318B_REG_SYSREF_DIV_0          0x00FD  /* SYSREF Divider byte 0 */
#define LMK05318B_REG_SYSREF_DIV_1          0x00FE  /* SYSREF Divider byte 1 */
#define LMK05318B_REG_SYSREF_DIV_2          0x00FF  /* SYSREF Divider byte 2 */

/* SYSREF Control Register (0xFC) Bit Definitions */
#define LMK05318B_SYSREF_DPLL_REF_SYNC_EN   (1 << 0)  /* DPLL Reference SYNC Enable */
#define LMK05318B_SYSREF_OUT7_EN            (1 << 1)  /* SYSREF on OUT7 Enable */
#define LMK05318B_SYSREF_NDIV_RST_DIS       (1 << 2)  /* N Divider Reset Disable */

/*============================================================================
 * Phase Offset Registers
 *===========================================================================*/

#define LMK05318B_REG_DPLL_PH_OFFSET_0      0x0154  /* DPLL Phase Offset byte 0 */
#define LMK05318B_REG_DPLL_PH_OFFSET_1      0x0155  /* DPLL Phase Offset byte 1 */
#define LMK05318B_REG_DPLL_PH_OFFSET_2      0x0156  /* DPLL Phase Offset byte 2 */
#define LMK05318B_REG_DPLL_PH_OFFSET_3      0x0157  /* DPLL Phase Offset byte 3 */
#define LMK05318B_REG_DPLL_PH_OFFSET_4      0x0158  /* DPLL Phase Offset byte 4 */
#define LMK05318B_REG_DPLL_PH_OFFSET_5      0x0159  /* DPLL Phase Offset byte 5 */

/*============================================================================
 * EEPROM Registers
 *===========================================================================*/

#define LMK05318B_REG_EEPROM_CTRL           0x01E0  /* EEPROM Control */
#define LMK05318B_REG_EEPROM_STATUS         0x01E1  /* EEPROM Status */
#define LMK05318B_REG_EEPROM_DATA_0         0x01E2  /* EEPROM Data byte 0 */
#define LMK05318B_REG_EEPROM_DATA_1         0x01E3  /* EEPROM Data byte 1 */
#define LMK05318B_REG_EEPROM_ADDR_0         0x01E4  /* EEPROM Address byte 0 */
#define LMK05318B_REG_EEPROM_ADDR_1         0x01E5  /* EEPROM Address byte 1 */

/* EEPROM Control Register Bit Definitions */
#define LMK05318B_EEPROM_CTRL_READ          (1 << 0)  /* EEPROM Read */
#define LMK05318B_EEPROM_CTRL_WRITE         (1 << 1)  /* EEPROM Write */
#define LMK05318B_EEPROM_CTRL_ERASE         (1 << 2)  /* EEPROM Erase */
#define LMK05318B_EEPROM_CTRL_LOCK          (1 << 3)  /* EEPROM Lock */

/* EEPROM Status Register Bit Definitions */
#define LMK05318B_EEPROM_STATUS_BUSY        (1 << 0)  /* EEPROM Busy */
#define LMK05318B_EEPROM_STATUS_ERROR       (1 << 1)  /* EEPROM Error */
#define LMK05318B_EEPROM_STATUS_LOCKED      (1 << 2)  /* EEPROM Locked */

/*============================================================================
 * I2C Address Configuration
 *===========================================================================*/

/* Default 7-bit I2C address: 0b1100_1xx where xx = GPIO[1:0] */
#define LMK05318B_I2C_ADDR_BASE             0x64    /* Base address (GPIO[1:0] = 00) */
#define LMK05318B_I2C_ADDR_GPIO_MASK        0x03    /* GPIO address bits mask */

/* Common I2C addresses based on GPIO configuration */
#define LMK05318B_I2C_ADDR_0x64             0x64    /* GPIO[1:0] = 00 */
#define LMK05318B_I2C_ADDR_0x65             0x65    /* GPIO[1:0] = 01 */
#define LMK05318B_I2C_ADDR_0x66             0x66    /* GPIO[1:0] = 10 */
#define LMK05318B_I2C_ADDR_0x67             0x67    /* GPIO[1:0] = 11 */

/*============================================================================
 * Timing Constants
 *===========================================================================*/

#define LMK05318B_RESET_DELAY_MS            10      /* Delay after software reset */
#define LMK05318B_PLL_LOCK_TIMEOUT_MS       100     /* PLL lock timeout */
#define LMK05318B_EEPROM_WRITE_DELAY_MS     10      /* EEPROM write delay */

/*============================================================================
 * Helper Macros
 *===========================================================================*/

/* Calculate channel register address */
#define LMK05318B_CH_REG(ch, offset) \
    (LMK05318B_REG_CH0_BASE + ((ch) * 8) + (offset))

/* Extract status bits */
#define LMK05318B_IS_PLL_LOCKED(status) \
    (((status) & (LMK05318B_STATUS_PLL1_LOCK | LMK05318B_STATUS_PLL2_LOCK)) == \
     (LMK05318B_STATUS_PLL1_LOCK | LMK05318B_STATUS_PLL2_LOCK))

#define LMK05318B_IS_DPLL_LOCKED(status) \
    ((status) & LMK05318B_STATUS_DPLL_LOCK)

#ifdef __cplusplus
}
#endif

#endif /* LMK05318B_REGS_H */
