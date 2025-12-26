/**
 * @file lmk05318b.c
 * @brief Texas Instruments LMK05318B Driver Implementation
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "lmk05318b.h"

/*============================================================================
 * Internal Helper Functions
 *===========================================================================*/

static inline bool is_initialized(lmk05318b_state_t *state)
{
    return state && state->initialized;
}

static inline void delay_ms(lmk05318b_state_t *state, uint32_t ms)
{
    if (state->delay) {
        state->delay(state->user_data, ms);
    }
}

/*============================================================================
 * Register Access Implementation
 *===========================================================================*/

lmk05318b_error_t lmk05318b_reg_read(lmk05318b_state_t *state,
                                      uint16_t reg_addr, uint8_t *value)
{
    if (!state || !value) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    if (!state->i2c_read) {
        return LMK05318B_ERR_I2C_COMM;
    }
    
    int ret = state->i2c_read(state->user_data, state->i2c_addr, reg_addr, value, 1);
    if (ret < 0) {
        return LMK05318B_ERR_I2C_COMM;
    }
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_reg_write(lmk05318b_state_t *state,
                                       uint16_t reg_addr, uint8_t value)
{
    if (!state) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    if (!state->i2c_write) {
        return LMK05318B_ERR_I2C_COMM;
    }
    
    int ret = state->i2c_write(state->user_data, state->i2c_addr, reg_addr, &value, 1);
    if (ret < 0) {
        return LMK05318B_ERR_I2C_COMM;
    }
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_reg_modify(lmk05318b_state_t *state,
                                        uint16_t reg_addr, uint8_t mask, uint8_t value)
{
    uint8_t current;
    lmk05318b_error_t err;
    
    err = lmk05318b_reg_read(state, reg_addr, &current);
    if (err != LMK05318B_OK) {
        return err;
    }
    
    current = (current & ~mask) | (value & mask);
    
    return lmk05318b_reg_write(state, reg_addr, current);
}

/*============================================================================
 * Initialization and Configuration
 *===========================================================================*/

lmk05318b_error_t lmk05318b_init(lmk05318b_state_t *state,
                                  uint8_t i2c_addr,
                                  void *user_data,
                                  lmk05318b_i2c_read_fn i2c_read,
                                  lmk05318b_i2c_write_fn i2c_write,
                                  lmk05318b_delay_fn delay)
{
    if (!state || !i2c_read || !i2c_write) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    /* Clear state */
    memset(state, 0, sizeof(lmk05318b_state_t));
    
    /* Store configuration */
    state->i2c_addr = i2c_addr;
    state->user_data = user_data;
    state->i2c_read = i2c_read;
    state->i2c_write = i2c_write;
    state->delay = delay;
    
    /* Verify device is present */
    lmk05318b_error_t err = lmk05318b_read_device_id(state, &state->device_id);
    if (err != LMK05318B_OK) {
        return LMK05318B_ERR_DEVICE_NOT_FOUND;
    }
    
    /* Verify it's the correct device */
    if (state->device_id.vendor_id != LMK05318B_VENDOR_ID) {
        return LMK05318B_ERR_DEVICE_NOT_FOUND;
    }
    
    state->initialized = true;
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_deinit(lmk05318b_state_t *state)
{
    if (!state) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    state->initialized = false;
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_reset(lmk05318b_state_t *state)
{
    if (!is_initialized(state)) {
        return LMK05318B_ERR_NOT_INITIALIZED;
    }
    
    /* Trigger software reset */
    lmk05318b_error_t err = lmk05318b_reg_write(state, LMK05318B_REG_RESET_SW,
                                                 LMK05318B_RESET_SW_SOFT_RST);
    if (err != LMK05318B_OK) {
        return err;
    }
    
    /* Wait for reset to complete */
    delay_ms(state, LMK05318B_RESET_DELAY_MS);
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_read_device_id(lmk05318b_state_t *state,
                                            lmk05318b_device_id_t *device_id)
{
    if (!state || !device_id) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    uint8_t data[8];
    lmk05318b_error_t err;
    
    /* Read vendor ID */
    err = lmk05318b_reg_read(state, LMK05318B_REG_VENDOR_ID_LSB, &data[0]);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_read(state, LMK05318B_REG_VENDOR_ID_MSB, &data[1]);
    if (err != LMK05318B_OK) return err;
    
    device_id->vendor_id = (data[1] << 8) | data[0];
    
    /* Read product ID */
    err = lmk05318b_reg_read(state, LMK05318B_REG_PRODUCT_ID_LSB, &data[0]);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_read(state, LMK05318B_REG_PRODUCT_ID_MSB, &data[1]);
    if (err != LMK05318B_OK) return err;
    
    device_id->product_id = (data[1] << 8) | data[0];
    
    /* Read part ID */
    for (int i = 0; i < 4; i++) {
        err = lmk05318b_reg_read(state, LMK05318B_REG_PART_ID_0 + i, &data[i]);
        if (err != LMK05318B_OK) return err;
    }
    
    device_id->part_id = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
    
    /* Read EEPROM revision */
    err = lmk05318b_reg_read(state, LMK05318B_REG_EEPROM_REV, &device_id->eeprom_rev);
    if (err != LMK05318B_OK) return err;
    
    return LMK05318B_OK;
}

bool lmk05318b_verify_device(lmk05318b_state_t *state)
{
    if (!state) {
        return false;
    }
    
    lmk05318b_device_id_t device_id;
    if (lmk05318b_read_device_id(state, &device_id) != LMK05318B_OK) {
        return false;
    }
    
    return (device_id.vendor_id == LMK05318B_VENDOR_ID);
}

/*============================================================================
 * Status and Monitoring
 *===========================================================================*/

lmk05318b_error_t lmk05318b_get_status(lmk05318b_state_t *state,
                                        lmk05318b_status_t *status)
{
    if (!is_initialized(state) || !status) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    uint8_t status0, status1;
    lmk05318b_error_t err;
    
    err = lmk05318b_reg_read(state, LMK05318B_REG_STATUS_0, &status0);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_read(state, LMK05318B_REG_STATUS_1, &status1);
    if (err != LMK05318B_OK) return err;
    
    status->pll1_locked = (status0 & LMK05318B_STATUS_PLL1_LOCK) != 0;
    status->pll2_locked = (status0 & LMK05318B_STATUS_PLL2_LOCK) != 0;
    status->dpll_locked = (status0 & LMK05318B_STATUS_DPLL_LOCK) != 0;
    status->apll1_locked = (status0 & LMK05318B_STATUS_APLL1_LOCK) != 0;
    status->apll2_locked = (status0 & LMK05318B_STATUS_APLL2_LOCK) != 0;
    
    status->loss_of_phase_lock = (status1 & LMK05318B_STATUS_LOPL) != 0;
    status->loss_of_freq_lock = (status1 & LMK05318B_STATUS_LOFL) != 0;
    status->loss_of_lock = (status1 & LMK05318B_STATUS_LOL) != 0;
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_wait_for_lock(lmk05318b_state_t *state,
                                           uint32_t timeout_ms)
{
    if (!is_initialized(state)) {
        return LMK05318B_ERR_NOT_INITIALIZED;
    }
    
    uint32_t elapsed = 0;
    const uint32_t poll_interval = 10;
    
    while (elapsed < timeout_ms) {
        lmk05318b_status_t status;
        lmk05318b_error_t err = lmk05318b_get_status(state, &status);
        if (err != LMK05318B_OK) {
            return err;
        }
        
        if (status.pll1_locked && status.pll2_locked && status.dpll_locked) {
            return LMK05318B_OK;
        }
        
        delay_ms(state, poll_interval);
        elapsed += poll_interval;
    }
    
    return LMK05318B_ERR_TIMEOUT;
}

/*============================================================================
 * Clock Configuration
 *===========================================================================*/

lmk05318b_error_t lmk05318b_set_input_ref(lmk05318b_state_t *state,
                                           lmk05318b_input_sel_t input,
                                           double freq_hz)
{
    if (!is_initialized(state)) {
        return LMK05318B_ERR_NOT_INITIALIZED;
    }
    
    uint8_t input_sel;
    switch (input) {
        case LMK05318B_INPUT_PRIREF:
            input_sel = LMK05318B_INPUT_SEL_PRIREF;
            break;
        case LMK05318B_INPUT_SECREF:
            input_sel = LMK05318B_INPUT_SEL_SECREF;
            break;
        case LMK05318B_INPUT_AUTO:
            input_sel = LMK05318B_INPUT_SEL_AUTO;
            break;
        default:
            return LMK05318B_ERR_INVALID_PARAM;
    }
    
    lmk05318b_error_t err = lmk05318b_reg_write(state, LMK05318B_REG_INPUT_SEL, input_sel);
    if (err != LMK05318B_OK) {
        return err;
    }
    
    state->clock_config.input_freq = freq_hz;
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_calc_pll_config(double input_freq,
                                             double output_freq,
                                             lmk05318b_clock_config_t *config)
{
    if (!config || input_freq <= 0 || output_freq <= 0) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    config->input_freq = input_freq;
    config->output_freq = output_freq;
    
    /* Calculate integer N divider */
    double ratio = output_freq / input_freq;
    config->dpll_n_div = (uint32_t)ratio;
    
    /* Calculate fractional part */
    double frac = ratio - (double)config->dpll_n_div;
    
    /* Use 24-bit fractional resolution */
    const uint32_t frac_resolution = 0x1000000;  /* 2^24 */
    config->dpll_frac_den = frac_resolution;
    config->dpll_frac_num = (uint32_t)(frac * frac_resolution);
    
    /* Default PLL dividers */
    config->pll1_n_div = 1;
    config->pll2_n_div = 1;
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_configure_pll(lmk05318b_state_t *state,
                                           const lmk05318b_clock_config_t *config)
{
    if (!is_initialized(state) || !config) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    lmk05318b_error_t err;
    
    /* Write DPLL N divider (40-bit value split across 5 registers) */
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_N_DIV_0, 
                               config->dpll_n_div & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_N_DIV_1,
                               (config->dpll_n_div >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_N_DIV_2,
                               (config->dpll_n_div >> 16) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_N_DIV_3,
                               (config->dpll_n_div >> 24) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_N_DIV_4, 0);
    if (err != LMK05318B_OK) return err;
    
    /* Write fractional numerator */
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_NUM_0,
                               config->dpll_frac_num & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_NUM_1,
                               (config->dpll_frac_num >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_NUM_2,
                               (config->dpll_frac_num >> 16) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_NUM_3,
                               (config->dpll_frac_num >> 24) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    /* Write fractional denominator */
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_DEN_0,
                               config->dpll_frac_den & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_DEN_1,
                               (config->dpll_frac_den >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_DEN_2,
                               (config->dpll_frac_den >> 16) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_DPLL_FRAC_DEN_3,
                               (config->dpll_frac_den >> 24) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    /* Write PLL1 N divider */
    err = lmk05318b_reg_write(state, LMK05318B_REG_PLL1_N_DIV_LSB,
                               config->pll1_n_div & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_PLL1_N_DIV_MSB,
                               (config->pll1_n_div >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    /* Write PLL2 N divider */
    err = lmk05318b_reg_write(state, LMK05318B_REG_PLL2_N_DIV_LSB,
                               config->pll2_n_div & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_PLL2_N_DIV_MSB,
                               (config->pll2_n_div >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    /* Store configuration */
    memcpy(&state->clock_config, config, sizeof(lmk05318b_clock_config_t));
    
    return LMK05318B_OK;
}

/*============================================================================
 * Output Channel Control
 *===========================================================================*/

lmk05318b_error_t lmk05318b_configure_channel(lmk05318b_state_t *state,
                                               uint8_t channel,
                                               const lmk05318b_channel_config_t *config)
{
    if (!is_initialized(state) || !config || channel > 7) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    uint16_t base_addr = LMK05318B_REG_CH0_BASE + (channel * 8);
    lmk05318b_error_t err;
    
    /* Channel control */
    uint8_t ctrl = 0;
    if (config->enabled) ctrl |= LMK05318B_CH_CTRL_EN;
    if (config->muted) ctrl |= LMK05318B_CH_CTRL_MUTE;
    
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_CTRL_OFFSET, ctrl);
    if (err != LMK05318B_OK) return err;
    
    /* Divider */
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DIV_LSB_OFFSET,
                               config->divider & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DIV_MSB_OFFSET,
                               (config->divider >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    /* Output format */
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_FORMAT_OFFSET,
                               (uint8_t)config->format);
    if (err != LMK05318B_OK) return err;
    
    /* Delay */
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DELAY_LSB_OFFSET,
                               config->delay & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DELAY_MSB_OFFSET,
                               (config->delay >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    /* Phase */
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_PHASE_OFFSET,
                               config->phase);
    if (err != LMK05318B_OK) return err;
    
    /* Store configuration */
    memcpy(&state->channels[channel], config, sizeof(lmk05318b_channel_config_t));
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_channel_enable(lmk05318b_state_t *state,
                                            uint8_t channel, bool enable)
{
    if (!is_initialized(state) || channel > 7) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    uint16_t addr = LMK05318B_CH_REG(channel, LMK05318B_CH_CTRL_OFFSET);
    
    return lmk05318b_reg_modify(state, addr, LMK05318B_CH_CTRL_EN,
                                 enable ? LMK05318B_CH_CTRL_EN : 0);
}

lmk05318b_error_t lmk05318b_set_channel_divider(lmk05318b_state_t *state,
                                                 uint8_t channel, uint16_t divider)
{
    if (!is_initialized(state) || channel > 7) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    uint16_t base_addr = LMK05318B_REG_CH0_BASE + (channel * 8);
    lmk05318b_error_t err;
    
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DIV_LSB_OFFSET,
                               divider & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DIV_MSB_OFFSET,
                               (divider >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    state->channels[channel].divider = divider;
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_set_channel_format(lmk05318b_state_t *state,
                                                uint8_t channel,
                                                lmk05318b_output_format_t format)
{
    if (!is_initialized(state) || channel > 7) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    uint16_t addr = LMK05318B_CH_REG(channel, LMK05318B_CH_FORMAT_OFFSET);
    
    lmk05318b_error_t err = lmk05318b_reg_write(state, addr, (uint8_t)format);
    if (err == LMK05318B_OK) {
        state->channels[channel].format = format;
    }
    
    return err;
}

lmk05318b_error_t lmk05318b_set_channel_delay(lmk05318b_state_t *state,
                                               uint8_t channel, uint16_t delay_ps)
{
    if (!is_initialized(state) || channel > 7) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    uint16_t base_addr = LMK05318B_REG_CH0_BASE + (channel * 8);
    lmk05318b_error_t err;
    
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DELAY_LSB_OFFSET,
                               delay_ps & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, base_addr + LMK05318B_CH_DELAY_MSB_OFFSET,
                               (delay_ps >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    state->channels[channel].delay = delay_ps;
    
    return LMK05318B_OK;
}

/*============================================================================
 * Synchronization Control
 *===========================================================================*/

lmk05318b_error_t lmk05318b_sync_outputs(lmk05318b_state_t *state)
{
    if (!is_initialized(state)) {
        return LMK05318B_ERR_NOT_INITIALIZED;
    }
    
    lmk05318b_error_t err;
    
    /* Assert SYNC */
    err = lmk05318b_reg_modify(state, LMK05318B_REG_RESET_SW,
                                LMK05318B_RESET_SW_SYNC_SW,
                                LMK05318B_RESET_SW_SYNC_SW);
    if (err != LMK05318B_OK) return err;
    
    /* Small delay */
    delay_ms(state, 1);
    
    /* De-assert SYNC */
    err = lmk05318b_reg_modify(state, LMK05318B_REG_RESET_SW,
                                LMK05318B_RESET_SW_SYNC_SW, 0);
    if (err != LMK05318B_OK) return err;
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_set_sync_enable(lmk05318b_state_t *state,
                                             uint8_t channel_mask)
{
    if (!is_initialized(state)) {
        return LMK05318B_ERR_NOT_INITIALIZED;
    }
    
    return lmk05318b_reg_write(state, LMK05318B_REG_CH_SYNC_EN, channel_mask);
}

lmk05318b_error_t lmk05318b_configure_sysref(lmk05318b_state_t *state,
                                              bool enable, uint32_t divider)
{
    if (!is_initialized(state)) {
        return LMK05318B_ERR_NOT_INITIALIZED;
    }
    
    lmk05318b_error_t err;
    
    /* Configure SYSREF divider */
    err = lmk05318b_reg_write(state, LMK05318B_REG_SYSREF_DIV_0, divider & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_SYSREF_DIV_1, (divider >> 8) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    err = lmk05318b_reg_write(state, LMK05318B_REG_SYSREF_DIV_2, (divider >> 16) & 0xFF);
    if (err != LMK05318B_OK) return err;
    
    /* Enable/disable SYSREF on OUT7 */
    uint8_t ctrl = enable ? LMK05318B_SYSREF_OUT7_EN : 0;
    err = lmk05318b_reg_write(state, LMK05318B_REG_SYSREF_CTRL, ctrl);
    if (err != LMK05318B_OK) return err;
    
    return LMK05318B_OK;
}

/*============================================================================
 * EEPROM Operations
 *===========================================================================*/

lmk05318b_error_t lmk05318b_eeprom_read(lmk05318b_state_t *state,
                                         uint16_t addr, uint8_t *data, size_t len)
{
    if (!is_initialized(state) || !data) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    lmk05318b_error_t err;
    
    for (size_t i = 0; i < len; i++) {
        /* Set EEPROM address */
        err = lmk05318b_reg_write(state, LMK05318B_REG_EEPROM_ADDR_0, (addr + i) & 0xFF);
        if (err != LMK05318B_OK) return err;
        
        err = lmk05318b_reg_write(state, LMK05318B_REG_EEPROM_ADDR_1, ((addr + i) >> 8) & 0xFF);
        if (err != LMK05318B_OK) return err;
        
        /* Trigger read */
        err = lmk05318b_reg_write(state, LMK05318B_REG_EEPROM_CTRL, LMK05318B_EEPROM_CTRL_READ);
        if (err != LMK05318B_OK) return err;
        
        /* Wait for completion */
        delay_ms(state, 1);
        
        /* Read data */
        err = lmk05318b_reg_read(state, LMK05318B_REG_EEPROM_DATA_0, &data[i]);
        if (err != LMK05318B_OK) return err;
    }
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_eeprom_write(lmk05318b_state_t *state,
                                          uint16_t addr, const uint8_t *data, size_t len)
{
    if (!is_initialized(state) || !data) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    lmk05318b_error_t err;
    
    for (size_t i = 0; i < len; i++) {
        /* Set EEPROM address */
        err = lmk05318b_reg_write(state, LMK05318B_REG_EEPROM_ADDR_0, (addr + i) & 0xFF);
        if (err != LMK05318B_OK) return err;
        
        err = lmk05318b_reg_write(state, LMK05318B_REG_EEPROM_ADDR_1, ((addr + i) >> 8) & 0xFF);
        if (err != LMK05318B_OK) return err;
        
        /* Set data */
        err = lmk05318b_reg_write(state, LMK05318B_REG_EEPROM_DATA_0, data[i]);
        if (err != LMK05318B_OK) return err;
        
        /* Trigger write */
        err = lmk05318b_reg_write(state, LMK05318B_REG_EEPROM_CTRL, LMK05318B_EEPROM_CTRL_WRITE);
        if (err != LMK05318B_OK) return err;
        
        /* Wait for completion */
        delay_ms(state, LMK05318B_EEPROM_WRITE_DELAY_MS);
    }
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_save_to_eeprom(lmk05318b_state_t *state)
{
    /* This would require reading all configuration registers and writing to EEPROM */
    /* Implementation depends on specific EEPROM layout */
    return LMK05318B_ERR_NOT_INITIALIZED;  /* TODO: Implement */
}

lmk05318b_error_t lmk05318b_load_from_eeprom(lmk05318b_state_t *state)
{
    if (!is_initialized(state)) {
        return LMK05318B_ERR_NOT_INITIALIZED;
    }
    
    /* Trigger EEPROM load by performing a soft reset */
    return lmk05318b_reset(state);
}

/*============================================================================
 * Configuration File Support
 *===========================================================================*/

lmk05318b_error_t lmk05318b_load_config_array(lmk05318b_state_t *state,
                                               const uint32_t *regs, size_t num_regs)
{
    if (!is_initialized(state) || !regs) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    lmk05318b_error_t err;
    
    for (size_t i = 0; i < num_regs; i++) {
        uint16_t addr = (regs[i] >> 8) & 0xFFFF;
        uint8_t value = regs[i] & 0xFF;
        
        err = lmk05318b_reg_write(state, addr, value);
        if (err != LMK05318B_OK) {
            return err;
        }
    }
    
    return LMK05318B_OK;
}

lmk05318b_error_t lmk05318b_load_config_file(lmk05318b_state_t *state,
                                              const char *filename)
{
    if (!is_initialized(state) || !filename) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        return LMK05318B_ERR_INVALID_PARAM;
    }
    
    char line[256];
    lmk05318b_error_t err = LMK05318B_OK;
    
    while (fgets(line, sizeof(line), fp)) {
        /* Skip comments and empty lines */
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        
        /* Parse hex value (format: 0xADDRVV or ADDRVAL) */
        uint32_t value;
        if (sscanf(line, "0x%x", &value) == 1 || sscanf(line, "%x", &value) == 1) {
            uint16_t addr = (value >> 8) & 0xFFFF;
            uint8_t data = value & 0xFF;
            
            err = lmk05318b_reg_write(state, addr, data);
            if (err != LMK05318B_OK) {
                break;
            }
        }
    }
    
    fclose(fp);
    return err;
}
