/**
 * @file test_carrier.c
 * @brief Unit tests for xMASS carrier board interface
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "xmass_carrier.h"

/*============================================================================
 * Test Cases
 *===========================================================================*/

static void test_init(void)
{
    printf("Test: xmass_carrier_init... ");
    
    xmass_carrier_state_t state;
    xmass_error_t err = xmass_carrier_init(&state, XMASS_I2C_METHOD_LINUX);
    
    /* May fail if no hardware present, but should not crash */
    if (err == XMASS_OK) {
        assert(state.initialized == true);
        xmass_carrier_deinit(&state);
    }
    
    printf("PASS (err=%d)\n", err);
}

static void test_detect_modules(void)
{
    printf("Test: xmass_carrier_detect_modules... ");
    
    xmass_carrier_state_t state;
    memset(&state, 0, sizeof(state));
    
    int count = xmass_carrier_detect_modules(&state);
    
    printf("PASS (found %d modules)\n", count);
}

static void test_get_module_info(void)
{
    printf("Test: xmass_carrier_get_module_info... ");
    
    xmass_carrier_state_t state;
    memset(&state, 0, sizeof(state));
    
    /* Initialize module info */
    for (int i = 0; i < XMASS_NUM_XSDR_MODULES; i++) {
        state.xsdr[i].index = i;
        state.xsdr[i].present = false;
    }
    
    xmass_xsdr_info_t info;
    xmass_error_t err = xmass_carrier_get_module_info(&state, 0, &info);
    
    assert(err == XMASS_OK);
    assert(info.index == 0);
    
    printf("PASS\n");
}

static void test_lmk_accessible(void)
{
    printf("Test: xmass_carrier_lmk_accessible... ");
    
    xmass_carrier_state_t state;
    memset(&state, 0, sizeof(state));
    state.lmk_accessible = false;
    
    bool accessible = xmass_carrier_lmk_accessible(&state);
    assert(accessible == false);
    
    state.lmk_accessible = true;
    accessible = xmass_carrier_lmk_accessible(&state);
    assert(accessible == true);
    
    printf("PASS\n");
}

static void test_get_ref_clock(void)
{
    printf("Test: xmass_carrier_get_ref_clock... ");
    
    xmass_carrier_state_t state;
    memset(&state, 0, sizeof(state));
    
    /* Set default ref clock */
    for (int i = 0; i < XMASS_NUM_XSDR_MODULES; i++) {
        state.xsdr[i].ref_clock_freq = XMASS_REF_CLOCK_HZ;
    }
    
    double freq_hz;
    xmass_error_t err = xmass_carrier_get_ref_clock(&state, 0, &freq_hz);
    
    assert(err == XMASS_OK);
    assert(freq_hz == XMASS_REF_CLOCK_HZ);
    
    printf("PASS (%.2f MHz)\n", freq_hz / 1e6);
}

static void test_print_status(void)
{
    printf("Test: xmass_carrier_print_status... ");
    
    xmass_carrier_state_t state;
    memset(&state, 0, sizeof(state));
    
    state.initialized = true;
    state.lmk_accessible = false;
    state.num_modules_detected = 0;
    
    for (int i = 0; i < XMASS_NUM_XSDR_MODULES; i++) {
        state.xsdr[i].index = i;
        snprintf(state.xsdr[i].pcie_slot, sizeof(state.xsdr[i].pcie_slot),
                 "0000:0%d:00.0", i + 4);
        snprintf(state.xsdr[i].device_path, sizeof(state.xsdr[i].device_path),
                 "/dev/usdr%d", i);
        state.xsdr[i].present = false;
        state.xsdr[i].ref_clock_freq = XMASS_REF_CLOCK_HZ;
    }
    
    printf("\n");
    xmass_carrier_print_status(&state);
    
    printf("Test: xmass_carrier_print_status... PASS\n");
}

/*============================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    printf("\n=== xMASS Carrier Board Unit Tests ===\n\n");
    
    test_init();
    test_detect_modules();
    test_get_module_info();
    test_lmk_accessible();
    test_get_ref_clock();
    test_print_status();
    
    printf("\n=== All tests passed! ===\n\n");
    
    return 0;
}
