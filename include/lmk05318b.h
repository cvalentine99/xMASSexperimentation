/**
 * @file lmk05318b.h
 * @brief Texas Instruments LMK05318B Driver API
 * 
 * Driver for the LMK05318B Ultra-Low Jitter Network Synchronizer
 * Designed for xMASS carrier board integration
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#ifndef LMK05318B_H
#define LMK05318B_H

#include <stdint.h>
#include <stdbool.h>
#include "lmk05318b_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Error Codes
 *===========================================================================*/

typedef enum {
    LMK05318B_OK = 0,
    LMK05318B_ERR_INVALID_PARAM = -1,
    LMK05318B_ERR_I2C_COMM = -2,
    LMK05318B_ERR_DEVICE_NOT_FOUND = -3,
    LMK05318B_ERR_PLL_NOT_LOCKED = -4,
    LMK05318B_ERR_TIMEOUT = -5,
    LMK05318B_ERR_EEPROM = -6,
    LMK05318B_ERR_NOT_INITIALIZED = -7,
    LMK05318B_ERR_BUSY = -8,
} lmk05318b_error_t;

/*============================================================================
 * Output Format Enumeration
 *===========================================================================*/

typedef enum {
    LMK05318B_OUTPUT_AC_LVPECL = 0,
    LMK05318B_OUTPUT_AC_CML = 1,
    LMK05318B_OUTPUT_AC_LVDS = 2,
    LMK05318B_OUTPUT_HSCL = 3,
    LMK05318B_OUTPUT_LVCMOS_1V8 = 4,
} lmk05318b_output_format_t;

/*============================================================================
 * Input Reference Selection
 *===========================================================================*/

typedef enum {
    LMK05318B_INPUT_PRIREF = 0,
    LMK05318B_INPUT_SECREF = 1,
    LMK05318B_INPUT_AUTO = 2,
} lmk05318b_input_sel_t;

/*============================================================================
 * PLL Status Structure
 *===========================================================================*/

typedef struct {
    bool pll1_locked;
    bool pll2_locked;
    bool dpll_locked;
    bool apll1_locked;
    bool apll2_locked;
    bool loss_of_phase_lock;
    bool loss_of_freq_lock;
    bool loss_of_lock;
} lmk05318b_status_t;

/*============================================================================
 * Device Identification Structure
 *===========================================================================*/

typedef struct {
    uint16_t vendor_id;
    uint16_t product_id;
    uint32_t part_id;
    uint8_t eeprom_rev;
} lmk05318b_device_id_t;

/*============================================================================
 * Output Channel Configuration
 *===========================================================================*/

typedef struct {
    bool enabled;
    bool muted;
    uint16_t divider;               /* Output divider value */
    lmk05318b_output_format_t format;
    uint16_t delay;                 /* Fine delay adjustment */
    uint8_t phase;                  /* Phase adjustment */
    bool sync_enabled;              /* SYNC enable for this channel */
} lmk05318b_channel_config_t;

/*============================================================================
 * Clock Configuration Structure
 *===========================================================================*/

typedef struct {
    double input_freq;              /* Input reference frequency (Hz) */
    double output_freq;             /* Desired output frequency (Hz) */
    uint32_t dpll_n_div;            /* DPLL N divider */
    uint32_t dpll_frac_num;         /* DPLL fractional numerator */
    uint32_t dpll_frac_den;         /* DPLL fractional denominator */
    uint16_t pll1_n_div;            /* PLL1 N divider */
    uint16_t pll2_n_div;            /* PLL2 N divider */
} lmk05318b_clock_config_t;

/*============================================================================
 * I2C Interface Abstraction
 *===========================================================================*/

/**
 * @brief I2C read function pointer type
 * @param user_data User-provided context
 * @param i2c_addr 7-bit I2C address
 * @param reg_addr 16-bit register address
 * @param data Pointer to store read data
 * @param len Number of bytes to read
 * @return 0 on success, negative error code on failure
 */
typedef int (*lmk05318b_i2c_read_fn)(void *user_data, uint8_t i2c_addr,
                                     uint16_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief I2C write function pointer type
 * @param user_data User-provided context
 * @param i2c_addr 7-bit I2C address
 * @param reg_addr 16-bit register address
 * @param data Pointer to data to write
 * @param len Number of bytes to write
 * @return 0 on success, negative error code on failure
 */
typedef int (*lmk05318b_i2c_write_fn)(void *user_data, uint8_t i2c_addr,
                                      uint16_t reg_addr, const uint8_t *data, size_t len);

/**
 * @brief Delay function pointer type
 * @param user_data User-provided context
 * @param ms Milliseconds to delay
 */
typedef void (*lmk05318b_delay_fn)(void *user_data, uint32_t ms);

/*============================================================================
 * Device State Structure
 *===========================================================================*/

typedef struct {
    /* I2C interface */
    uint8_t i2c_addr;
    void *user_data;
    lmk05318b_i2c_read_fn i2c_read;
    lmk05318b_i2c_write_fn i2c_write;
    lmk05318b_delay_fn delay;
    
    /* Device state */
    bool initialized;
    lmk05318b_device_id_t device_id;
    
    /* Current configuration */
    lmk05318b_clock_config_t clock_config;
    lmk05318b_channel_config_t channels[8];
    
} lmk05318b_state_t;

/*============================================================================
 * Initialization and Configuration
 *===========================================================================*/

/**
 * @brief Initialize the LMK05318B driver
 * @param state Pointer to driver state structure
 * @param i2c_addr 7-bit I2C address of the device
 * @param user_data User-provided context for I2C callbacks
 * @param i2c_read I2C read function
 * @param i2c_write I2C write function
 * @param delay Delay function
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_init(lmk05318b_state_t *state,
                                  uint8_t i2c_addr,
                                  void *user_data,
                                  lmk05318b_i2c_read_fn i2c_read,
                                  lmk05318b_i2c_write_fn i2c_write,
                                  lmk05318b_delay_fn delay);

/**
 * @brief Deinitialize the LMK05318B driver
 * @param state Pointer to driver state structure
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_deinit(lmk05318b_state_t *state);

/**
 * @brief Perform software reset
 * @param state Pointer to driver state structure
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_reset(lmk05318b_state_t *state);

/**
 * @brief Read device identification
 * @param state Pointer to driver state structure
 * @param device_id Pointer to store device ID
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_read_device_id(lmk05318b_state_t *state,
                                            lmk05318b_device_id_t *device_id);

/**
 * @brief Verify device is LMK05318B
 * @param state Pointer to driver state structure
 * @return true if device is valid LMK05318B, false otherwise
 */
bool lmk05318b_verify_device(lmk05318b_state_t *state);

/*============================================================================
 * Register Access
 *===========================================================================*/

/**
 * @brief Read a single register
 * @param state Pointer to driver state structure
 * @param reg_addr 16-bit register address
 * @param value Pointer to store read value
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_reg_read(lmk05318b_state_t *state,
                                      uint16_t reg_addr, uint8_t *value);

/**
 * @brief Write a single register
 * @param state Pointer to driver state structure
 * @param reg_addr 16-bit register address
 * @param value Value to write
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_reg_write(lmk05318b_state_t *state,
                                       uint16_t reg_addr, uint8_t value);

/**
 * @brief Modify register bits using mask
 * @param state Pointer to driver state structure
 * @param reg_addr 16-bit register address
 * @param mask Bit mask
 * @param value New value for masked bits
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_reg_modify(lmk05318b_state_t *state,
                                        uint16_t reg_addr, uint8_t mask, uint8_t value);

/*============================================================================
 * Status and Monitoring
 *===========================================================================*/

/**
 * @brief Read device status
 * @param state Pointer to driver state structure
 * @param status Pointer to store status
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_get_status(lmk05318b_state_t *state,
                                        lmk05318b_status_t *status);

/**
 * @brief Wait for PLL lock
 * @param state Pointer to driver state structure
 * @param timeout_ms Timeout in milliseconds
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_wait_for_lock(lmk05318b_state_t *state,
                                           uint32_t timeout_ms);

/*============================================================================
 * Clock Configuration
 *===========================================================================*/

/**
 * @brief Configure input reference
 * @param state Pointer to driver state structure
 * @param input Input reference selection
 * @param freq_hz Input frequency in Hz
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_set_input_ref(lmk05318b_state_t *state,
                                           lmk05318b_input_sel_t input,
                                           double freq_hz);

/**
 * @brief Configure PLL for desired output frequency
 * @param state Pointer to driver state structure
 * @param config Clock configuration structure
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_configure_pll(lmk05318b_state_t *state,
                                           const lmk05318b_clock_config_t *config);

/**
 * @brief Calculate PLL configuration for given frequencies
 * @param input_freq Input reference frequency in Hz
 * @param output_freq Desired output frequency in Hz
 * @param config Pointer to store calculated configuration
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_calc_pll_config(double input_freq,
                                             double output_freq,
                                             lmk05318b_clock_config_t *config);

/*============================================================================
 * Output Channel Control
 *===========================================================================*/

/**
 * @brief Configure an output channel
 * @param state Pointer to driver state structure
 * @param channel Channel number (0-7)
 * @param config Channel configuration
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_configure_channel(lmk05318b_state_t *state,
                                               uint8_t channel,
                                               const lmk05318b_channel_config_t *config);

/**
 * @brief Enable/disable an output channel
 * @param state Pointer to driver state structure
 * @param channel Channel number (0-7)
 * @param enable true to enable, false to disable
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_channel_enable(lmk05318b_state_t *state,
                                            uint8_t channel, bool enable);

/**
 * @brief Set output channel divider
 * @param state Pointer to driver state structure
 * @param channel Channel number (0-7)
 * @param divider Divider value
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_set_channel_divider(lmk05318b_state_t *state,
                                                 uint8_t channel, uint16_t divider);

/**
 * @brief Set output channel format
 * @param state Pointer to driver state structure
 * @param channel Channel number (0-7)
 * @param format Output format
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_set_channel_format(lmk05318b_state_t *state,
                                                uint8_t channel,
                                                lmk05318b_output_format_t format);

/**
 * @brief Set output channel delay
 * @param state Pointer to driver state structure
 * @param channel Channel number (0-7)
 * @param delay_ps Delay in picoseconds
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_set_channel_delay(lmk05318b_state_t *state,
                                               uint8_t channel, uint16_t delay_ps);

/*============================================================================
 * Synchronization Control
 *===========================================================================*/

/**
 * @brief Trigger SYNC event (phase align all outputs)
 * @param state Pointer to driver state structure
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_sync_outputs(lmk05318b_state_t *state);

/**
 * @brief Enable SYNC for specific channels
 * @param state Pointer to driver state structure
 * @param channel_mask Bitmask of channels to enable SYNC
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_set_sync_enable(lmk05318b_state_t *state,
                                             uint8_t channel_mask);

/**
 * @brief Configure SYSREF output on CH7
 * @param state Pointer to driver state structure
 * @param enable Enable SYSREF output
 * @param divider SYSREF divider value
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_configure_sysref(lmk05318b_state_t *state,
                                              bool enable, uint32_t divider);

/*============================================================================
 * EEPROM Operations
 *===========================================================================*/

/**
 * @brief Read from EEPROM
 * @param state Pointer to driver state structure
 * @param addr EEPROM address
 * @param data Pointer to store read data
 * @param len Number of bytes to read
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_eeprom_read(lmk05318b_state_t *state,
                                         uint16_t addr, uint8_t *data, size_t len);

/**
 * @brief Write to EEPROM
 * @param state Pointer to driver state structure
 * @param addr EEPROM address
 * @param data Pointer to data to write
 * @param len Number of bytes to write
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_eeprom_write(lmk05318b_state_t *state,
                                          uint16_t addr, const uint8_t *data, size_t len);

/**
 * @brief Save current configuration to EEPROM
 * @param state Pointer to driver state structure
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_save_to_eeprom(lmk05318b_state_t *state);

/**
 * @brief Load configuration from EEPROM
 * @param state Pointer to driver state structure
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_load_from_eeprom(lmk05318b_state_t *state);

/*============================================================================
 * Configuration File Support
 *===========================================================================*/

/**
 * @brief Load configuration from TICS Pro hex file
 * @param state Pointer to driver state structure
 * @param filename Path to hex configuration file
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_load_config_file(lmk05318b_state_t *state,
                                              const char *filename);

/**
 * @brief Load configuration from register array
 * @param state Pointer to driver state structure
 * @param regs Array of register address/value pairs
 * @param num_regs Number of register pairs
 * @return LMK05318B_OK on success, error code on failure
 */
lmk05318b_error_t lmk05318b_load_config_array(lmk05318b_state_t *state,
                                               const uint32_t *regs, size_t num_regs);

#ifdef __cplusplus
}
#endif

#endif /* LMK05318B_H */
