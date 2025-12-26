/**
 * @file xmass_carrier.c
 * @brief xMASS Carrier Board Interface Implementation
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "xmass_carrier.h"

/*============================================================================
 * Internal Constants
 *===========================================================================*/

/* PCIe slot addresses for xSDR modules */
static const char *XSDR_PCIE_SLOTS[XMASS_NUM_XSDR_MODULES] = {
    "0000:04:00.0",
    "0000:05:00.0",
    "0000:06:00.0",
    "0000:07:00.0"
};

/* Device paths */
static const char *XSDR_DEV_PATHS[XMASS_NUM_XSDR_MODULES] = {
    "/dev/usdr0",
    "/dev/usdr1",
    "/dev/usdr2",
    "/dev/usdr3"
};

/*============================================================================
 * Internal I2C Callback Functions for LMK05318B
 *===========================================================================*/

static int carrier_i2c_read_cb(void *user_data, uint8_t i2c_addr,
                                uint16_t reg_addr, uint8_t *data, size_t len)
{
    xmass_carrier_state_t *state = (xmass_carrier_state_t *)user_data;
    
    if (xmass_carrier_i2c_read(state, i2c_addr, reg_addr, data, len) != XMASS_OK) {
        return -1;
    }
    return 0;
}

static int carrier_i2c_write_cb(void *user_data, uint8_t i2c_addr,
                                 uint16_t reg_addr, const uint8_t *data, size_t len)
{
    xmass_carrier_state_t *state = (xmass_carrier_state_t *)user_data;
    
    if (xmass_carrier_i2c_write(state, i2c_addr, reg_addr, data, len) != XMASS_OK) {
        return -1;
    }
    return 0;
}

static void carrier_delay_cb(void *user_data, uint32_t ms)
{
    (void)user_data;
    usleep(ms * 1000);
}

/*============================================================================
 * Initialization and Configuration
 *===========================================================================*/

xmass_error_t xmass_carrier_init(xmass_carrier_state_t *state,
                                  xmass_i2c_method_t i2c_method)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    /* Clear state */
    memset(state, 0, sizeof(xmass_carrier_state_t));
    state->i2c_fd = -1;
    
    /* Initialize xSDR module info */
    for (int i = 0; i < XMASS_NUM_XSDR_MODULES; i++) {
        state->xsdr[i].index = i;
        strncpy(state->xsdr[i].pcie_slot, XSDR_PCIE_SLOTS[i], 
                sizeof(state->xsdr[i].pcie_slot) - 1);
        strncpy(state->xsdr[i].device_path, XSDR_DEV_PATHS[i],
                sizeof(state->xsdr[i].device_path) - 1);
        state->xsdr[i].ref_clock_freq = XMASS_REF_CLOCK_HZ;
    }
    
    /* Detect modules */
    int num_detected = xmass_carrier_detect_modules(state);
    if (num_detected < 0) {
        return XMASS_ERR_NOT_FOUND;
    }
    
    /* Try to open I2C based on method */
    xmass_error_t err = XMASS_OK;
    
    switch (i2c_method) {
        case XMASS_I2C_METHOD_LINUX:
            /* Try common I2C bus numbers */
            for (int bus = 0; bus < 10; bus++) {
                err = xmass_carrier_i2c_open(state, bus);
                if (err == XMASS_OK) {
                    break;
                }
            }
            break;
            
        case XMASS_I2C_METHOD_USDR:
            /* Will use usdr-lib for I2C access */
            state->i2c_bus = 0;
            err = XMASS_OK;
            break;
            
        case XMASS_I2C_METHOD_PCIE_BAR:
            /* Map PCIe BAR for direct access */
            err = xmass_carrier_pcie_map(state, 0);
            break;
            
        case XMASS_I2C_METHOD_SYSFS:
            /* Use sysfs attributes */
            err = XMASS_OK;
            break;
            
        default:
            return XMASS_ERR_INVALID_PARAM;
    }
    
    /* Try to initialize LMK05318B */
    state->lmk_accessible = false;
    
    lmk05318b_error_t lmk_err = lmk05318b_init(&state->lmk,
                                                XMASS_LMK05318B_I2C_ADDR,
                                                state,
                                                carrier_i2c_read_cb,
                                                carrier_i2c_write_cb,
                                                carrier_delay_cb);
    
    if (lmk_err == LMK05318B_OK) {
        state->lmk_accessible = true;
        printf("[xMASS] LMK05318B clock synthesizer accessible\n");
    } else {
        printf("[xMASS] LMK05318B not accessible (error %d) - using software sync\n", lmk_err);
    }
    
    state->initialized = true;
    state->lo_freq_hz = XMASS_DEFAULT_LO_FREQ_HZ;
    
    return XMASS_OK;
}

xmass_error_t xmass_carrier_deinit(xmass_carrier_state_t *state)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    /* Deinitialize LMK05318B */
    if (state->lmk_accessible) {
        lmk05318b_deinit(&state->lmk);
    }
    
    /* Close I2C */
    xmass_carrier_i2c_close(state);
    
    /* Unmap PCIe */
    xmass_carrier_pcie_unmap(state);
    
    state->initialized = false;
    
    return XMASS_OK;
}

int xmass_carrier_detect_modules(xmass_carrier_state_t *state)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    int count = 0;
    
    for (int i = 0; i < XMASS_NUM_XSDR_MODULES; i++) {
        /* Check if device file exists */
        if (access(state->xsdr[i].device_path, F_OK) == 0) {
            state->xsdr[i].present = true;
            count++;
            
            /* Try to read hardware ID */
            char sysfs_path[256];
            snprintf(sysfs_path, sizeof(sysfs_path),
                     "/sys/class/usdr/usdr%d/device/vendor", i);
            
            FILE *fp = fopen(sysfs_path, "r");
            if (fp) {
                uint32_t vendor;
                if (fscanf(fp, "0x%x", &vendor) == 1) {
                    /* Vendor ID read successfully */
                }
                fclose(fp);
            }
            
            printf("[xMASS] Detected xSDR module %d at %s\n", 
                   i, state->xsdr[i].pcie_slot);
        } else {
            state->xsdr[i].present = false;
        }
    }
    
    state->num_modules_detected = count;
    
    return count;
}

xmass_error_t xmass_carrier_get_module_info(xmass_carrier_state_t *state,
                                             uint8_t index,
                                             xmass_xsdr_info_t *info)
{
    if (!state || !info || index >= XMASS_NUM_XSDR_MODULES) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    memcpy(info, &state->xsdr[index], sizeof(xmass_xsdr_info_t));
    
    return XMASS_OK;
}

/*============================================================================
 * LMK05318B Access
 *===========================================================================*/

bool xmass_carrier_lmk_accessible(xmass_carrier_state_t *state)
{
    return state && state->lmk_accessible;
}

lmk05318b_state_t* xmass_carrier_get_lmk(xmass_carrier_state_t *state)
{
    if (!state || !state->lmk_accessible) {
        return NULL;
    }
    return &state->lmk;
}

xmass_error_t xmass_carrier_probe_lmk(xmass_carrier_state_t *state)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    /* Try different I2C addresses */
    uint8_t addrs[] = {0x64, 0x65, 0x66, 0x67};
    
    for (size_t i = 0; i < sizeof(addrs); i++) {
        lmk05318b_error_t err = lmk05318b_init(&state->lmk,
                                                addrs[i],
                                                state,
                                                carrier_i2c_read_cb,
                                                carrier_i2c_write_cb,
                                                carrier_delay_cb);
        if (err == LMK05318B_OK) {
            state->lmk_accessible = true;
            printf("[xMASS] Found LMK05318B at I2C address 0x%02X\n", addrs[i]);
            return XMASS_OK;
        }
    }
    
    /* Try through each xSDR module's I2C buses */
    for (int mod = 0; mod < XMASS_NUM_XSDR_MODULES; mod++) {
        if (!state->xsdr[mod].present) continue;
        
        for (int bus = 0; bus < 3; bus++) {
            printf("[xMASS] Probing LMK05318B via xSDR%d I2C bus %d...\n", mod, bus);
            
            /* Try to read device ID through usdr-lib */
            uint8_t data[4];
            xmass_error_t err = xmass_carrier_usdr_i2c_op(state, mod, bus,
                                                          XMASS_LMK05318B_I2C_ADDR,
                                                          0x0000, data, 4, false);
            if (err == XMASS_OK) {
                uint16_t vendor_id = (data[1] << 8) | data[0];
                if (vendor_id == LMK05318B_VENDOR_ID) {
                    printf("[xMASS] Found LMK05318B via xSDR%d bus %d\n", mod, bus);
                    state->lmk_accessible = true;
                    return XMASS_OK;
                }
            }
        }
    }
    
    return XMASS_ERR_LMK_NOT_ACCESSIBLE;
}

/*============================================================================
 * I2C Bus Operations
 *===========================================================================*/

xmass_error_t xmass_carrier_i2c_open(xmass_carrier_state_t *state, uint8_t bus_num)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", bus_num);
    
    state->i2c_fd = open(path, O_RDWR);
    if (state->i2c_fd < 0) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    state->i2c_bus = bus_num;
    
    return XMASS_OK;
}

xmass_error_t xmass_carrier_i2c_close(xmass_carrier_state_t *state)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    if (state->i2c_fd >= 0) {
        close(state->i2c_fd);
        state->i2c_fd = -1;
    }
    
    return XMASS_OK;
}

int xmass_carrier_i2c_scan(xmass_carrier_state_t *state,
                           uint8_t *devices, size_t max_devices)
{
    if (!state || !devices || state->i2c_fd < 0) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    int count = 0;
    
    for (uint8_t addr = 0x08; addr < 0x78 && count < (int)max_devices; addr++) {
        if (ioctl(state->i2c_fd, I2C_SLAVE, addr) < 0) {
            continue;
        }
        
        /* Try to read a byte */
        uint8_t dummy;
        if (read(state->i2c_fd, &dummy, 1) == 1) {
            devices[count++] = addr;
        }
    }
    
    return count;
}

xmass_error_t xmass_carrier_i2c_read(xmass_carrier_state_t *state,
                                      uint8_t addr, uint16_t reg_addr,
                                      uint8_t *data, size_t len)
{
    if (!state || !data) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    if (state->i2c_fd < 0) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    /* Set slave address */
    if (ioctl(state->i2c_fd, I2C_SLAVE, addr) < 0) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    /* Write register address (16-bit, big-endian) */
    uint8_t reg_buf[2] = {(reg_addr >> 8) & 0xFF, reg_addr & 0xFF};
    
    struct i2c_msg msgs[2] = {
        {
            .addr = addr,
            .flags = 0,
            .len = 2,
            .buf = reg_buf
        },
        {
            .addr = addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = data
        }
    };
    
    struct i2c_rdwr_ioctl_data ioctl_data = {
        .msgs = msgs,
        .nmsgs = 2
    };
    
    if (ioctl(state->i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    return XMASS_OK;
}

xmass_error_t xmass_carrier_i2c_write(xmass_carrier_state_t *state,
                                       uint8_t addr, uint16_t reg_addr,
                                       const uint8_t *data, size_t len)
{
    if (!state || !data) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    if (state->i2c_fd < 0) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    /* Set slave address */
    if (ioctl(state->i2c_fd, I2C_SLAVE, addr) < 0) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    /* Build write buffer: [addr_hi, addr_lo, data...] */
    uint8_t *buf = malloc(2 + len);
    if (!buf) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    buf[0] = (reg_addr >> 8) & 0xFF;
    buf[1] = reg_addr & 0xFF;
    memcpy(buf + 2, data, len);
    
    struct i2c_msg msg = {
        .addr = addr,
        .flags = 0,
        .len = 2 + len,
        .buf = buf
    };
    
    struct i2c_rdwr_ioctl_data ioctl_data = {
        .msgs = &msg,
        .nmsgs = 1
    };
    
    int ret = ioctl(state->i2c_fd, I2C_RDWR, &ioctl_data);
    free(buf);
    
    if (ret < 0) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    return XMASS_OK;
}

/*============================================================================
 * PCIe BAR Access
 *===========================================================================*/

xmass_error_t xmass_carrier_pcie_map(xmass_carrier_state_t *state,
                                      uint8_t module_index)
{
    if (!state || module_index >= XMASS_NUM_XSDR_MODULES) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    /* PCIe BAR mapping would require root privileges and specific BAR addresses */
    /* This is a placeholder - actual implementation depends on hardware */
    
    return XMASS_ERR_UNSUPPORTED;
}

xmass_error_t xmass_carrier_pcie_unmap(xmass_carrier_state_t *state)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    return XMASS_OK;
}

xmass_error_t xmass_carrier_pcie_read32(xmass_carrier_state_t *state,
                                         uint32_t offset, uint32_t *value)
{
    (void)state;
    (void)offset;
    (void)value;
    return XMASS_ERR_UNSUPPORTED;
}

xmass_error_t xmass_carrier_pcie_write32(xmass_carrier_state_t *state,
                                          uint32_t offset, uint32_t value)
{
    (void)state;
    (void)offset;
    (void)value;
    return XMASS_ERR_UNSUPPORTED;
}

/*============================================================================
 * usdr-lib Integration
 *===========================================================================*/

xmass_error_t xmass_carrier_usdr_i2c_op(xmass_carrier_state_t *state,
                                         uint8_t module_index,
                                         uint8_t i2c_bus,
                                         uint8_t i2c_addr,
                                         uint16_t reg_addr,
                                         uint8_t *data, size_t len,
                                         bool write)
{
    if (!state || !data || module_index >= XMASS_NUM_XSDR_MODULES) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    if (!state->xsdr[module_index].present) {
        return XMASS_ERR_NOT_FOUND;
    }
    
    /*
     * This would use the usdr-lib lowlevel API:
     * 
     * #include <usdr_lowlevel.h>
     * 
     * uint32_t lsaddr = MAKE_LSOP_I2C_ADDR(0, i2c_bus, i2c_addr);
     * 
     * if (write) {
     *     uint8_t buf[3] = {reg_addr >> 8, reg_addr & 0xFF, *data};
     *     return lowlevel_ls_op(lldev, 0, USDR_LSOP_I2C_DEV, lsaddr,
     *                           0, NULL, 3, buf);
     * } else {
     *     uint8_t addr_buf[2] = {reg_addr >> 8, reg_addr & 0xFF};
     *     return lowlevel_ls_op(lldev, 0, USDR_LSOP_I2C_DEV, lsaddr,
     *                           len, data, 2, addr_buf);
     * }
     */
    
    /* Placeholder - requires actual usdr-lib integration */
    (void)i2c_bus;
    (void)i2c_addr;
    (void)reg_addr;
    (void)len;
    (void)write;
    
    return XMASS_ERR_UNSUPPORTED;
}

/*============================================================================
 * Clock Distribution
 *===========================================================================*/

xmass_error_t xmass_carrier_get_ref_clock(xmass_carrier_state_t *state,
                                           uint8_t module_index,
                                           double *freq_hz)
{
    if (!state || !freq_hz || module_index >= XMASS_NUM_XSDR_MODULES) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    /* Try to read from sysfs */
    char path[256];
    snprintf(path, sizeof(path),
             "/sys/class/usdr/usdr%d/dm/sdr/refclk/frequency", module_index);
    
    FILE *fp = fopen(path, "r");
    if (fp) {
        uint32_t freq;
        if (fscanf(fp, "%u", &freq) == 1) {
            *freq_hz = (double)freq;
            fclose(fp);
            return XMASS_OK;
        }
        fclose(fp);
    }
    
    /* Return default if sysfs not available */
    *freq_hz = state->xsdr[module_index].ref_clock_freq;
    
    return XMASS_OK;
}

xmass_error_t xmass_carrier_set_ref_clock(xmass_carrier_state_t *state,
                                           double freq_hz)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    if (!state->lmk_accessible) {
        return XMASS_ERR_LMK_NOT_ACCESSIBLE;
    }
    
    /* Calculate PLL configuration */
    lmk05318b_clock_config_t config;
    lmk05318b_error_t err = lmk05318b_calc_pll_config(100e6, freq_hz, &config);
    if (err != LMK05318B_OK) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    /* Configure PLL */
    err = lmk05318b_configure_pll(&state->lmk, &config);
    if (err != LMK05318B_OK) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    /* Wait for lock */
    err = lmk05318b_wait_for_lock(&state->lmk, LMK05318B_PLL_LOCK_TIMEOUT_MS);
    if (err != LMK05318B_OK) {
        return XMASS_ERR_TIMEOUT;
    }
    
    /* Update stored frequency */
    for (int i = 0; i < XMASS_NUM_XSDR_MODULES; i++) {
        state->xsdr[i].ref_clock_freq = freq_hz;
    }
    
    return XMASS_OK;
}

xmass_error_t xmass_carrier_sync_clocks(xmass_carrier_state_t *state)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    if (!state->lmk_accessible) {
        return XMASS_ERR_LMK_NOT_ACCESSIBLE;
    }
    
    /* Enable SYNC on all channels */
    lmk05318b_error_t err = lmk05318b_set_sync_enable(&state->lmk, LMK05318B_CH_SYNC_ALL);
    if (err != LMK05318B_OK) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    /* Trigger SYNC */
    err = lmk05318b_sync_outputs(&state->lmk);
    if (err != LMK05318B_OK) {
        return XMASS_ERR_I2C_ACCESS;
    }
    
    printf("[xMASS] Clock outputs synchronized\n");
    
    return XMASS_OK;
}

xmass_error_t xmass_carrier_set_lo_frequency(xmass_carrier_state_t *state,
                                              double freq_hz)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }

    if (!state->lmk_accessible) {
        return XMASS_ERR_LMK_NOT_ACCESSIBLE;
    }

    printf("[xMASS] Setting LO frequency to %.3f MHz\n", freq_hz / 1e6);

    /* This assumes LO is generated from one of the LMK outputs and distributed.
     * We will configure channels 4-7 for the LO signal.
     */
    lmk05318b_channel_config_t lo_ch_config = {
        .enabled = true,
        .muted = false,
        .divider = 4, /* Example: 104MHz / 4 = 26MHz. This needs to be calculated. */
        .format = LMK05318B_OUTPUT_AC_LVDS,
        .delay = 0,
        .phase = 0
    };

    /* Calculate divider based on a VCO frequency. Assume 2.5GHz VCO for now. */
    uint16_t divider = (uint16_t)(2500e6 / freq_hz);
    lo_ch_config.divider = divider;

    lmk05318b_error_t err;
    for (int i = 0; i < 4; i++) {
        err = lmk05318b_configure_channel(&state->lmk, 4 + i, &lo_ch_config);
        if (err != LMK05318B_OK) {
            return XMASS_ERR_I2C_ACCESS;
        }
    }

    /* Synchronize the outputs */
    err = lmk05318b_sync_outputs(&state->lmk);
    if (err != LMK05318B_OK) {
        return XMASS_ERR_I2C_ACCESS;
    }

    state->lo_freq_hz = freq_hz;

    return XMASS_OK;
}

xmass_error_t xmass_carrier_get_lo_frequency(xmass_carrier_state_t *state,
                                              double *freq_hz)
{
    if (!state || !freq_hz) {
        return XMASS_ERR_INVALID_PARAM;
    }

    *freq_hz = state->lo_freq_hz;

    return XMASS_OK;
}

/*============================================================================
 * Diagnostics
 *===========================================================================*/

void xmass_carrier_print_status(xmass_carrier_state_t *state)
{
    if (!state) {
        printf("[xMASS] State is NULL\n");
        return;
    }
    
    printf("\n=== xMASS Carrier Board Status ===\n\n");
    
    printf("Initialized: %s\n", state->initialized ? "Yes" : "No");
    printf("LMK05318B accessible: %s\n", state->lmk_accessible ? "Yes" : "No");
    printf("Modules detected: %d\n\n", state->num_modules_detected);
    
    printf("xSDR Modules:\n");
    printf("  Index | PCIe Slot       | Device        | Present | Ref Clock\n");
    printf("  ------|-----------------|---------------|---------|----------\n");
    
    for (int i = 0; i < XMASS_NUM_XSDR_MODULES; i++) {
        printf("  %d     | %s | %s | %s   | %.2f MHz\n",
               state->xsdr[i].index,
               state->xsdr[i].pcie_slot,
               state->xsdr[i].device_path,
               state->xsdr[i].present ? "Yes" : "No",
               state->xsdr[i].ref_clock_freq / 1e6);
    }
    
    if (state->lmk_accessible) {
        printf("\nLMK05318B Status:\n");
        lmk05318b_status_t status;
        if (lmk05318b_get_status(&state->lmk, &status) == LMK05318B_OK) {
            printf("  PLL1 Locked: %s\n", status.pll1_locked ? "Yes" : "No");
            printf("  PLL2 Locked: %s\n", status.pll2_locked ? "Yes" : "No");
            printf("  DPLL Locked: %s\n", status.dpll_locked ? "Yes" : "No");
            printf("  Loss of Lock: %s\n", status.loss_of_lock ? "Yes" : "No");
        }
        
        lmk05318b_device_id_t dev_id;
        if (lmk05318b_read_device_id(&state->lmk, &dev_id) == LMK05318B_OK) {
            printf("  Vendor ID: 0x%04X\n", dev_id.vendor_id);
            printf("  Product ID: 0x%04X\n", dev_id.product_id);
            printf("  Part ID: 0x%08X\n", dev_id.part_id);
        }
    }
    
    printf("\n");
}

xmass_error_t xmass_carrier_dump_registers(xmass_carrier_state_t *state,
                                            const char *filename)
{
    if (!state) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    FILE *fp = filename ? fopen(filename, "w") : stdout;
    if (!fp) {
        return XMASS_ERR_INVALID_PARAM;
    }
    
    fprintf(fp, "# xMASS Register Dump\n\n");
    
    if (state->lmk_accessible) {
        fprintf(fp, "## LMK05318B Registers\n\n");
        fprintf(fp, "Address | Value\n");
        fprintf(fp, "--------|------\n");
        
        /* Dump key registers */
        uint16_t regs[] = {
            0x0000, 0x0001, 0x0002, 0x0003,  /* Device ID */
            0x0008, 0x000C, 0x000D, 0x000E,  /* Control/Status */
            0x0020, 0x0030, 0x0040, 0x0046, 0x0047,  /* Input/PLL/SYNC */
            0x0050, 0x0058, 0x0060, 0x0068,  /* Channels 0-3 */
            0x0070, 0x0078, 0x0080, 0x0088   /* Channels 4-7 */
        };
        
        for (size_t i = 0; i < sizeof(regs)/sizeof(regs[0]); i++) {
            uint8_t value;
            if (lmk05318b_reg_read(&state->lmk, regs[i], &value) == LMK05318B_OK) {
                fprintf(fp, "0x%04X  | 0x%02X\n", regs[i], value);
            }
        }
    }
    
    if (filename) {
        fclose(fp);
    }
    
    return XMASS_OK;
}
