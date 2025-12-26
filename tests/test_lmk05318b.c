/**
 * @file test_lmk05318b.c
 * @brief Unit tests for LMK05318B driver
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "lmk05318b.h"

/*============================================================================
 * Mock I2C Implementation
 *===========================================================================*/

static uint8_t mock_registers[0x200];
static bool mock_initialized = false;

static void mock_init(void)
{
    memset(mock_registers, 0, sizeof(mock_registers));
    
    /* Set up device ID registers */
    mock_registers[LMK05318B_REG_VENDOR_ID_LSB] = LMK05318B_VENDOR_ID & 0xFF;
    mock_registers[LMK05318B_REG_VENDOR_ID_MSB] = (LMK05318B_VENDOR_ID >> 8) & 0xFF;
    mock_registers[LMK05318B_REG_PRODUCT_ID_LSB] = 0x18;
    mock_registers[LMK05318B_REG_PRODUCT_ID_MSB] = 0x53;
    mock_registers[LMK05318B_REG_PART_ID_0] = 0x01;
    mock_registers[LMK05318B_REG_PART_ID_1] = 0x02;
    mock_registers[LMK05318B_REG_PART_ID_2] = 0x03;
    mock_registers[LMK05318B_REG_PART_ID_3] = 0x04;
    mock_registers[LMK05318B_REG_EEPROM_REV] = 0x01;
    
    /* Set up status registers (all PLLs locked) */
    mock_registers[LMK05318B_REG_STATUS_0] = LMK05318B_STATUS_PLL1_LOCK |
                                              LMK05318B_STATUS_PLL2_LOCK |
                                              LMK05318B_STATUS_DPLL_LOCK |
                                              LMK05318B_STATUS_APLL1_LOCK |
                                              LMK05318B_STATUS_APLL2_LOCK;
    mock_registers[LMK05318B_REG_STATUS_1] = 0;
    
    mock_initialized = true;
}

static int mock_i2c_read(void *user_data, uint8_t i2c_addr,
                          uint16_t reg_addr, uint8_t *data, size_t len)
{
    (void)user_data;
    (void)i2c_addr;
    
    if (!mock_initialized) mock_init();
    
    if (reg_addr >= sizeof(mock_registers)) {
        return -1;
    }
    
    for (size_t i = 0; i < len && (reg_addr + i) < sizeof(mock_registers); i++) {
        data[i] = mock_registers[reg_addr + i];
    }
    
    return 0;
}

static int mock_i2c_write(void *user_data, uint8_t i2c_addr,
                           uint16_t reg_addr, const uint8_t *data, size_t len)
{
    (void)user_data;
    (void)i2c_addr;
    
    if (!mock_initialized) mock_init();
    
    if (reg_addr >= sizeof(mock_registers)) {
        return -1;
    }
    
    for (size_t i = 0; i < len && (reg_addr + i) < sizeof(mock_registers); i++) {
        mock_registers[reg_addr + i] = data[i];
    }
    
    return 0;
}

static void mock_delay(void *user_data, uint32_t ms)
{
    (void)user_data;
    (void)ms;
    /* No actual delay in tests */
}

/*============================================================================
 * Test Cases
 *===========================================================================*/

static void test_init(void)
{
    printf("Test: lmk05318b_init... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_error_t err = lmk05318b_init(&state, 0x65, NULL,
                                            mock_i2c_read, mock_i2c_write, mock_delay);
    
    assert(err == LMK05318B_OK);
    assert(state.initialized == true);
    assert(state.device_id.vendor_id == LMK05318B_VENDOR_ID);
    
    printf("PASS\n");
}

static void test_device_id(void)
{
    printf("Test: lmk05318b_read_device_id... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_init(&state, 0x65, NULL, mock_i2c_read, mock_i2c_write, mock_delay);
    
    lmk05318b_device_id_t device_id;
    lmk05318b_error_t err = lmk05318b_read_device_id(&state, &device_id);
    
    assert(err == LMK05318B_OK);
    assert(device_id.vendor_id == LMK05318B_VENDOR_ID);
    assert(device_id.product_id == 0x5318);
    assert(device_id.part_id == 0x04030201);
    assert(device_id.eeprom_rev == 0x01);
    
    printf("PASS\n");
}

static void test_status(void)
{
    printf("Test: lmk05318b_get_status... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_init(&state, 0x65, NULL, mock_i2c_read, mock_i2c_write, mock_delay);
    
    lmk05318b_status_t status;
    lmk05318b_error_t err = lmk05318b_get_status(&state, &status);
    
    assert(err == LMK05318B_OK);
    assert(status.pll1_locked == true);
    assert(status.pll2_locked == true);
    assert(status.dpll_locked == true);
    assert(status.loss_of_lock == false);
    
    printf("PASS\n");
}

static void test_register_rw(void)
{
    printf("Test: register read/write... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_init(&state, 0x65, NULL, mock_i2c_read, mock_i2c_write, mock_delay);
    
    /* Write a value */
    lmk05318b_error_t err = lmk05318b_reg_write(&state, 0x0100, 0xAB);
    assert(err == LMK05318B_OK);
    
    /* Read it back */
    uint8_t value;
    err = lmk05318b_reg_read(&state, 0x0100, &value);
    assert(err == LMK05318B_OK);
    assert(value == 0xAB);
    
    printf("PASS\n");
}

static void test_register_modify(void)
{
    printf("Test: register modify... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_init(&state, 0x65, NULL, mock_i2c_read, mock_i2c_write, mock_delay);
    
    /* Set initial value */
    lmk05318b_reg_write(&state, 0x0100, 0xFF);
    
    /* Modify only lower nibble */
    lmk05318b_error_t err = lmk05318b_reg_modify(&state, 0x0100, 0x0F, 0x05);
    assert(err == LMK05318B_OK);
    
    /* Verify */
    uint8_t value;
    lmk05318b_reg_read(&state, 0x0100, &value);
    assert(value == 0xF5);
    
    printf("PASS\n");
}

static void test_channel_config(void)
{
    printf("Test: channel configuration... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_init(&state, 0x65, NULL, mock_i2c_read, mock_i2c_write, mock_delay);
    
    lmk05318b_channel_config_t config = {
        .enabled = true,
        .muted = false,
        .divider = 4,
        .format = LMK05318B_OUTPUT_AC_LVDS,
        .delay = 100,
        .phase = 0
    };
    
    lmk05318b_error_t err = lmk05318b_configure_channel(&state, 0, &config);
    assert(err == LMK05318B_OK);
    
    /* Verify channel is enabled */
    uint8_t ctrl;
    lmk05318b_reg_read(&state, LMK05318B_REG_CH0_BASE + LMK05318B_CH_CTRL_OFFSET, &ctrl);
    assert((ctrl & LMK05318B_CH_CTRL_EN) != 0);
    
    printf("PASS\n");
}

static void test_pll_config(void)
{
    printf("Test: PLL configuration calculation... ");
    
    lmk05318b_clock_config_t config;
    lmk05318b_error_t err = lmk05318b_calc_pll_config(100e6, 26e6, &config);
    
    assert(err == LMK05318B_OK);
    assert(config.input_freq == 100e6);
    assert(config.output_freq == 26e6);
    assert(config.dpll_n_div > 0);
    
    printf("PASS\n");
}

static void test_sync(void)
{
    printf("Test: sync outputs... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_init(&state, 0x65, NULL, mock_i2c_read, mock_i2c_write, mock_delay);
    
    lmk05318b_error_t err = lmk05318b_sync_outputs(&state);
    assert(err == LMK05318B_OK);
    
    printf("PASS\n");
}

static void test_wait_for_lock(void)
{
    printf("Test: wait for lock... ");
    
    mock_init();
    
    lmk05318b_state_t state;
    lmk05318b_init(&state, 0x65, NULL, mock_i2c_read, mock_i2c_write, mock_delay);
    
    /* PLLs are already locked in mock */
    lmk05318b_error_t err = lmk05318b_wait_for_lock(&state, 1000);
    assert(err == LMK05318B_OK);
    
    printf("PASS\n");
}

/*============================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    printf("\n=== LMK05318B Driver Unit Tests ===\n\n");
    
    test_init();
    test_device_id();
    test_status();
    test_register_rw();
    test_register_modify();
    test_channel_config();
    test_pll_config();
    test_sync();
    test_wait_for_lock();
    
    printf("\n=== All tests passed! ===\n\n");
    
    return 0;
}
