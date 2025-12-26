/**
 * @file xmass_carrier.h
 * @brief xMASS Carrier Board Interface
 * 
 * Interface for accessing carrier board peripherals including the LMK05318B
 * clock synthesizer through PCIe BAR registers and I2C passthrough.
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#ifndef XMASS_CARRIER_H
#define XMASS_CARRIER_H

#include <stdint.h>
#include <stdbool.h>
#include "lmk05318b.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants
 *===========================================================================*/

#define XMASS_NUM_XSDR_MODULES      4       /* Number of xSDR modules */
#define XMASS_CARRIER_I2C_BUS       0       /* Carrier board I2C bus number */
#define XMASS_LMK05318B_I2C_ADDR    0x65    /* LMK05318B I2C address on carrier */

/* PCIe device identifiers */
#define XMASS_XSDR_VENDOR_ID        0x10EE  /* Xilinx */
#define XMASS_XSDR_DEVICE_ID        0x7049  /* xSDR FPGA */

/* Reference clock frequency from LMK05318B */
#define XMASS_REF_CLOCK_HZ          26000000    /* 26 MHz */

/* Default LO frequency */
#define XMASS_DEFAULT_LO_FREQ_HZ    100000000   /* 100 MHz */

/*============================================================================
 * Error Codes
 *===========================================================================*/

typedef enum {
    XMASS_OK = 0,
    XMASS_ERR_INVALID_PARAM = -1,
    XMASS_ERR_NOT_FOUND = -2,
    XMASS_ERR_I2C_ACCESS = -3,
    XMASS_ERR_PCIE_ACCESS = -4,
    XMASS_ERR_NOT_INITIALIZED = -5,
    XMASS_ERR_DEVICE_BUSY = -6,
    XMASS_ERR_TIMEOUT = -7,
    XMASS_ERR_LMK_NOT_ACCESSIBLE = -8,
    XMASS_ERR_UNSUPPORTED = -9,
} xmass_error_t;

/*============================================================================
 * xSDR Module Information
 *===========================================================================*/

typedef struct {
    uint8_t index;                  /* Module index (0-3) */
    char pcie_slot[32];             /* PCIe slot identifier (e.g., "0000:04:00.0") */
    char device_path[64];           /* Device path (e.g., "/dev/usdr0") */
    uint32_t hardware_id;           /* Hardware ID from device */
    bool present;                   /* Module is present and detected */
    bool initialized;               /* Module has been initialized */
    double ref_clock_freq;          /* Reference clock frequency (Hz) */
} xmass_xsdr_info_t;

/*============================================================================
 * Carrier Board State
 *===========================================================================*/

typedef struct {
    /* LMK05318B clock synthesizer */
    lmk05318b_state_t lmk;
    bool lmk_accessible;            /* True if LMK05318B can be accessed */
    
    /* xSDR modules */
    xmass_xsdr_info_t xsdr[XMASS_NUM_XSDR_MODULES];
    int num_modules_detected;
    
    /* I2C interface */
    int i2c_fd;                     /* File descriptor for I2C bus */
    uint8_t i2c_bus;                /* I2C bus number */
    
    /* State */
    bool initialized;
    
    /* User context for callbacks */
    void *user_data;

    /* Frequencies */
    double lo_freq_hz;
    
} xmass_carrier_state_t;

/*============================================================================
 * I2C Access Methods
 *===========================================================================*/

/**
 * @brief I2C access method enumeration
 */
typedef enum {
    XMASS_I2C_METHOD_LINUX,         /* Linux /dev/i2c-N interface */
    XMASS_I2C_METHOD_USDR,          /* Through usdr-lib lowlevel API */
    XMASS_I2C_METHOD_PCIE_BAR,      /* Direct PCIe BAR register access */
    XMASS_I2C_METHOD_SYSFS,         /* Through sysfs attributes */
} xmass_i2c_method_t;

/*============================================================================
 * Initialization and Configuration
 *===========================================================================*/

/**
 * @brief Initialize the xMASS carrier board interface
 * @param state Pointer to carrier state structure
 * @param i2c_method I2C access method to use
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_init(xmass_carrier_state_t *state,
                                  xmass_i2c_method_t i2c_method);

/**
 * @brief Deinitialize the xMASS carrier board interface
 * @param state Pointer to carrier state structure
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_deinit(xmass_carrier_state_t *state);

/**
 * @brief Detect all xSDR modules on the carrier board
 * @param state Pointer to carrier state structure
 * @return Number of modules detected, or negative error code
 */
int xmass_carrier_detect_modules(xmass_carrier_state_t *state);

/**
 * @brief Get information about a specific xSDR module
 * @param state Pointer to carrier state structure
 * @param index Module index (0-3)
 * @param info Pointer to store module information
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_get_module_info(xmass_carrier_state_t *state,
                                             uint8_t index,
                                             xmass_xsdr_info_t *info);

/*============================================================================
 * LMK05318B Access
 *===========================================================================*/

/**
 * @brief Check if LMK05318B is accessible
 * @param state Pointer to carrier state structure
 * @return true if accessible, false otherwise
 */
bool xmass_carrier_lmk_accessible(xmass_carrier_state_t *state);

/**
 * @brief Get pointer to LMK05318B driver state
 * @param state Pointer to carrier state structure
 * @return Pointer to LMK05318B state, or NULL if not accessible
 */
lmk05318b_state_t* xmass_carrier_get_lmk(xmass_carrier_state_t *state);

/**
 * @brief Try to access LMK05318B through alternative methods
 * @param state Pointer to carrier state structure
 * @return XMASS_OK if access established, error code otherwise
 */
xmass_error_t xmass_carrier_probe_lmk(xmass_carrier_state_t *state);

/*============================================================================
 * I2C Bus Operations
 *===========================================================================*/

/**
 * @brief Open I2C bus for carrier board access
 * @param state Pointer to carrier state structure
 * @param bus_num I2C bus number
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_i2c_open(xmass_carrier_state_t *state, uint8_t bus_num);

/**
 * @brief Close I2C bus
 * @param state Pointer to carrier state structure
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_i2c_close(xmass_carrier_state_t *state);

/**
 * @brief Scan I2C bus for devices
 * @param state Pointer to carrier state structure
 * @param devices Array to store found device addresses
 * @param max_devices Maximum number of devices to return
 * @return Number of devices found, or negative error code
 */
int xmass_carrier_i2c_scan(xmass_carrier_state_t *state,
                           uint8_t *devices, size_t max_devices);

/**
 * @brief Read from I2C device
 * @param state Pointer to carrier state structure
 * @param addr 7-bit I2C address
 * @param reg_addr Register address (16-bit for LMK05318B)
 * @param data Pointer to store read data
 * @param len Number of bytes to read
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_i2c_read(xmass_carrier_state_t *state,
                                      uint8_t addr, uint16_t reg_addr,
                                      uint8_t *data, size_t len);

/**
 * @brief Write to I2C device
 * @param state Pointer to carrier state structure
 * @param addr 7-bit I2C address
 * @param reg_addr Register address (16-bit for LMK05318B)
 * @param data Pointer to data to write
 * @param len Number of bytes to write
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_i2c_write(xmass_carrier_state_t *state,
                                       uint8_t addr, uint16_t reg_addr,
                                       const uint8_t *data, size_t len);

/*============================================================================
 * PCIe BAR Access (for I2C passthrough)
 *===========================================================================*/

/**
 * @brief Map PCIe BAR for direct register access
 * @param state Pointer to carrier state structure
 * @param module_index xSDR module index (0-3)
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_pcie_map(xmass_carrier_state_t *state,
                                      uint8_t module_index);

/**
 * @brief Unmap PCIe BAR
 * @param state Pointer to carrier state structure
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_pcie_unmap(xmass_carrier_state_t *state);

/**
 * @brief Read from PCIe BAR register
 * @param state Pointer to carrier state structure
 * @param offset Register offset within BAR
 * @param value Pointer to store read value
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_pcie_read32(xmass_carrier_state_t *state,
                                         uint32_t offset, uint32_t *value);

/**
 * @brief Write to PCIe BAR register
 * @param state Pointer to carrier state structure
 * @param offset Register offset within BAR
 * @param value Value to write
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_pcie_write32(xmass_carrier_state_t *state,
                                          uint32_t offset, uint32_t value);

/*============================================================================
 * usdr-lib Integration
 *===========================================================================*/

/**
 * @brief Access I2C through usdr-lib lowlevel API
 * @param state Pointer to carrier state structure
 * @param module_index xSDR module to use for access
 * @param i2c_bus I2C bus number within module
 * @param i2c_addr 7-bit I2C address
 * @param reg_addr Register address
 * @param data Data buffer
 * @param len Data length
 * @param write true for write, false for read
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_usdr_i2c_op(xmass_carrier_state_t *state,
                                         uint8_t module_index,
                                         uint8_t i2c_bus,
                                         uint8_t i2c_addr,
                                         uint16_t reg_addr,
                                         uint8_t *data, size_t len,
                                         bool write);

/*============================================================================
 * Clock Distribution
 *===========================================================================*/

/**
 * @brief Get reference clock frequency for a module
 * @param state Pointer to carrier state structure
 * @param module_index Module index (0-3)
 * @param freq_hz Pointer to store frequency in Hz
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_get_ref_clock(xmass_carrier_state_t *state,
                                           uint8_t module_index,
                                           double *freq_hz);

/**
 * @brief Configure reference clock distribution
 * @param state Pointer to carrier state structure
 * @param freq_hz Desired reference frequency in Hz
 * @return XMASS_OK on success, error code on failure
 * @note Requires LMK05318B access
 */
xmass_error_t xmass_carrier_set_ref_clock(xmass_carrier_state_t *state,
                                           double freq_hz);

/**
 * @brief Configure LO frequency distribution
 * @param state Pointer to carrier state structure
 * @param freq_hz Desired LO frequency in Hz
 * @return XMASS_OK on success, error code on failure
 * @note Requires LMK05318B access
 */
xmass_error_t xmass_carrier_set_lo_frequency(xmass_carrier_state_t *state,
                                              double freq_hz);

/**
 * @brief Get current LO frequency
 * @param state Pointer to carrier state structure
 * @param freq_hz Pointer to store frequency in Hz
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_get_lo_frequency(xmass_carrier_state_t *state,
                                              double *freq_hz);

/**
 * @brief Synchronize all clock outputs
 * @param state Pointer to carrier state structure
 * @return XMASS_OK on success, error code on failure
 * @note Requires LMK05318B access
 */
xmass_error_t xmass_carrier_sync_clocks(xmass_carrier_state_t *state);

/*============================================================================
 * Diagnostics
 *===========================================================================*/

/**
 * @brief Print carrier board status
 * @param state Pointer to carrier state structure
 */
void xmass_carrier_print_status(xmass_carrier_state_t *state);

/**
 * @brief Dump all accessible registers for debugging
 * @param state Pointer to carrier state structure
 * @param filename Output file path (NULL for stdout)
 * @return XMASS_OK on success, error code on failure
 */
xmass_error_t xmass_carrier_dump_registers(xmass_carrier_state_t *state,
                                            const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* XMASS_CARRIER_H */
