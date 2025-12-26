/**
 * @file test_sync.c
 * @brief Unit tests for xMASS synchronization module
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include "xmass_sync.h"

#define PI 3.14159265358979323846

/*============================================================================
 * Test Cases
 *===========================================================================*/

static void test_buffer_alloc(void)
{
    printf("Test: xmass_sync_buffer_alloc... ");
    
    xmass_sync_buffer_t buffer;
    xmass_sync_error_t err = xmass_sync_buffer_alloc(&buffer, 1024);
    
    assert(err == XMASS_SYNC_OK);
    assert(buffer.samples != NULL);
    assert(buffer.num_samples == 1024);
    assert(buffer.valid == false);
    
    xmass_sync_buffer_free(&buffer);
    assert(buffer.samples == NULL);
    
    printf("PASS\n");
}

static void test_buffer_clear(void)
{
    printf("Test: xmass_sync_buffer_clear... ");
    
    xmass_sync_buffer_t buffer;
    xmass_sync_buffer_alloc(&buffer, 1024);
    
    /* Set some data */
    for (size_t i = 0; i < buffer.num_samples; i++) {
        buffer.samples[i] = 1.0f + 1.0f * I;
    }
    buffer.valid = true;
    
    xmass_sync_buffer_clear(&buffer);
    
    assert(buffer.valid == false);
    assert(crealf(buffer.samples[0]) == 0.0f);
    
    xmass_sync_buffer_free(&buffer);
    
    printf("PASS\n");
}

static void test_default_config(void)
{
    printf("Test: xmass_sync_get_default_config... ");
    
    xmass_sync_config_t config;
    xmass_sync_get_default_config(&config);
    
    assert(config.sample_rate == 1e6);
    assert(config.gain == 40.0);
    assert(config.num_samples == 8192);
    assert(config.calibration_freq == 100e6);
    
    printf("PASS\n");
}

static void test_error_string(void)
{
    printf("Test: xmass_sync_error_string... ");
    
    const char *str;
    
    str = xmass_sync_error_string(XMASS_SYNC_OK);
    assert(strcmp(str, "Success") == 0);
    
    str = xmass_sync_error_string(XMASS_SYNC_ERR_INVALID_PARAM);
    assert(strcmp(str, "Invalid parameter") == 0);
    
    str = xmass_sync_error_string(XMASS_SYNC_ERR_CALIBRATION_FAILED);
    assert(strcmp(str, "Calibration failed") == 0);
    
    printf("PASS\n");
}

static void test_init_deinit(void)
{
    printf("Test: xmass_sync_init/deinit... ");
    
    /* Create mock carrier state */
    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;
    
    xmass_sync_state_t state;
    xmass_sync_error_t err = xmass_sync_init(&state, &carrier);
    
    assert(err == XMASS_SYNC_OK);
    assert(state.initialized == true);
    assert(state.num_channels == 4);
    assert(state.calibration.valid == false);
    
    err = xmass_sync_deinit(&state);
    assert(err == XMASS_SYNC_OK);
    assert(state.initialized == false);
    
    printf("PASS\n");
}

static void test_configure(void)
{
    printf("Test: xmass_sync_configure... ");
    
    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;
    
    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);
    
    xmass_sync_config_t config;
    xmass_sync_get_default_config(&config);
    config.sample_rate = 2e6;
    config.gain = 50.0;
    
    xmass_sync_error_t err = xmass_sync_configure(&state, &config);
    
    assert(err == XMASS_SYNC_OK);
    assert(state.config.sample_rate == 2e6);
    assert(state.config.gain == 50.0);
    
    xmass_sync_deinit(&state);
    
    printf("PASS\n");
}

static void test_calibration_save_load(void)
{
    printf("Test: calibration save/load... ");
    
    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;
    
    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);
    
    /* Set calibration data */
    state.calibration.phase_offset[0] = 0.0;
    state.calibration.phase_offset[1] = PI / 4;
    state.calibration.phase_offset[2] = PI / 2;
    state.calibration.phase_offset[3] = 3 * PI / 4;
    
    state.calibration.amplitude_correction[0] = 1.0;
    state.calibration.amplitude_correction[1] = 1.1;
    state.calibration.amplitude_correction[2] = 0.9;
    state.calibration.amplitude_correction[3] = 1.05;
    
    state.calibration.calibration_freq = 100e6;
    state.calibration.valid = true;
    state.calibration.calibration_count = 1;
    
    /* Save */
    xmass_sync_error_t err = xmass_sync_save_calibration(&state, "/tmp/test_cal.txt");
    assert(err == XMASS_SYNC_OK);
    
    /* Clear and reload */
    memset(&state.calibration, 0, sizeof(state.calibration));
    
    err = xmass_sync_load_calibration(&state, "/tmp/test_cal.txt");
    assert(err == XMASS_SYNC_OK);
    
    /* Verify */
    assert(state.calibration.valid == true);
    assert(fabs(state.calibration.phase_offset[1] - PI / 4) < 1e-6);
    assert(fabs(state.calibration.amplitude_correction[1] - 1.1) < 1e-6);
    
    xmass_sync_deinit(&state);
    
    /* Cleanup */
    remove("/tmp/test_cal.txt");
    
    printf("PASS\n");
}

static void test_calibration_valid(void)
{
    printf("Test: xmass_sync_calibration_valid... ");
    
    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;
    
    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);
    
    /* Initially invalid */
    assert(xmass_sync_calibration_valid(&state) == false);
    
    /* Set valid calibration with recent timestamp */
    state.calibration.valid = true;
    state.calibration.calibration_timestamp = (uint64_t)time(NULL);
    state.config.calibration_interval = 3600;  /* 1 hour */
    
    assert(xmass_sync_calibration_valid(&state) == true);
    
    /* Set old timestamp */
    state.calibration.calibration_timestamp = (uint64_t)time(NULL) - 7200;  /* 2 hours ago */
    
    assert(xmass_sync_calibration_valid(&state) == false);
    
    xmass_sync_deinit(&state);
    
    printf("PASS\n");
}

static void test_print_status(void)
{
    printf("Test: xmass_sync_print_status... ");
    
    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;
    
    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);
    
    /* Set some calibration data */
    state.calibration.phase_offset[0] = 0.0;
    state.calibration.phase_offset[1] = 0.1;
    state.calibration.phase_offset[2] = 0.2;
    state.calibration.phase_offset[3] = 0.3;
    state.calibration.amplitude_correction[0] = 1.0;
    state.calibration.amplitude_correction[1] = 1.0;
    state.calibration.amplitude_correction[2] = 1.0;
    state.calibration.amplitude_correction[3] = 1.0;
    state.calibration.valid = true;
    state.calibration.calibration_timestamp = (uint64_t)time(NULL);
    
    printf("\n");
    xmass_sync_print_status(&state);
    
    xmass_sync_deinit(&state);
    
    printf("Test: xmass_sync_print_status... PASS\n");
}

/*============================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    printf("\n=== xMASS Synchronization Unit Tests ===\n\n");
    
    test_buffer_alloc();
    test_buffer_clear();
    test_default_config();
    test_error_string();
    test_init_deinit();
    test_configure();
    test_calibration_save_load();
    test_calibration_valid();
    test_print_status();
    
    printf("\n=== All tests passed! ===\n\n");
    
    return 0;
}
