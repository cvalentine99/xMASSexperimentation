/**
 * @file test_sync.c
 * @brief Unit tests for xMASS synchronization module
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>
#include "xmass_sync.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    state.calibration.phase_offset[1] = M_PI / 4;
    state.calibration.phase_offset[2] = M_PI / 2;
    state.calibration.phase_offset[3] = 3 * M_PI / 4;
    
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
    assert(fabs(state.calibration.phase_offset[1] - M_PI / 4) < 1e-6);
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
 * Additional Comprehensive Tests
 *===========================================================================*/

static void test_null_pointer_handling(void)
{
    printf("Test: null pointer handling... ");

    xmass_sync_error_t err;

    /* Test init with NULL state */
    err = xmass_sync_init(NULL, NULL);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    /* Test init with NULL carrier */
    xmass_sync_state_t state;
    err = xmass_sync_init(&state, NULL);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    /* Test deinit with NULL */
    err = xmass_sync_deinit(NULL);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    /* Test configure with NULL */
    err = xmass_sync_configure(NULL, NULL);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    /* Test buffer alloc with NULL */
    err = xmass_sync_buffer_alloc(NULL, 1024);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    /* Test get_default_config with NULL (should not crash) */
    xmass_sync_get_default_config(NULL);

    printf("PASS\n");
}

static void test_buffer_edge_cases(void)
{
    printf("Test: buffer edge cases... ");

    xmass_sync_buffer_t buffer;
    xmass_sync_error_t err;

    /* Zero samples should fail */
    err = xmass_sync_buffer_alloc(&buffer, 0);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    /* Very small buffer should work */
    err = xmass_sync_buffer_alloc(&buffer, 1);
    assert(err == XMASS_SYNC_OK);
    assert(buffer.num_samples == 1);
    xmass_sync_buffer_free(&buffer);

    /* Free NULL buffer should be safe */
    xmass_sync_buffer_free(NULL);

    /* Clear NULL buffer should be safe */
    xmass_sync_buffer_clear(NULL);

    /* Double free should be safe */
    err = xmass_sync_buffer_alloc(&buffer, 100);
    assert(err == XMASS_SYNC_OK);
    xmass_sync_buffer_free(&buffer);
    xmass_sync_buffer_free(&buffer);  /* Second free should be safe */

    printf("PASS\n");
}

static void test_calibration_get_set(void)
{
    printf("Test: calibration get/set... ");

    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;

    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);

    /* Create calibration data */
    xmass_sync_calibration_t cal_in;
    memset(&cal_in, 0, sizeof(cal_in));
    cal_in.phase_offset[0] = 0.0;
    cal_in.phase_offset[1] = 0.5;
    cal_in.phase_offset[2] = 1.0;
    cal_in.phase_offset[3] = 1.5;
    cal_in.amplitude_correction[0] = 1.0;
    cal_in.amplitude_correction[1] = 1.2;
    cal_in.amplitude_correction[2] = 0.8;
    cal_in.amplitude_correction[3] = 1.1;
    cal_in.calibration_freq = 150e6;
    cal_in.valid = true;
    cal_in.calibration_count = 5;

    /* Set calibration */
    xmass_sync_error_t err = xmass_sync_set_calibration(&state, &cal_in);
    assert(err == XMASS_SYNC_OK);

    /* Get calibration */
    xmass_sync_calibration_t cal_out;
    err = xmass_sync_get_calibration(&state, &cal_out);
    assert(err == XMASS_SYNC_OK);

    /* Verify */
    assert(cal_out.valid == true);
    assert(cal_out.calibration_count == 5);
    assert(fabs(cal_out.calibration_freq - 150e6) < 1e-3);
    for (int i = 0; i < XMASS_SYNC_MAX_CHANNELS; i++) {
        assert(fabs(cal_out.phase_offset[i] - cal_in.phase_offset[i]) < 1e-9);
        assert(fabs(cal_out.amplitude_correction[i] - cal_in.amplitude_correction[i]) < 1e-9);
    }

    /* Test with NULL parameters */
    err = xmass_sync_get_calibration(NULL, &cal_out);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    err = xmass_sync_get_calibration(&state, NULL);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    err = xmass_sync_set_calibration(NULL, &cal_in);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    err = xmass_sync_set_calibration(&state, NULL);
    assert(err == XMASS_SYNC_ERR_INVALID_PARAM);

    xmass_sync_deinit(&state);

    printf("PASS\n");
}

static void test_apply_corrections_without_calibration(void)
{
    printf("Test: apply_corrections without calibration... ");

    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;

    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);

    /* Allocate buffers */
    xmass_sync_buffer_t buffers[XMASS_SYNC_MAX_CHANNELS];
    for (int i = 0; i < XMASS_SYNC_MAX_CHANNELS; i++) {
        xmass_sync_buffer_alloc(&buffers[i], 100);
        buffers[i].valid = true;
    }

    /* Try to apply corrections without valid calibration */
    xmass_sync_error_t err = xmass_sync_apply_corrections(&state, buffers);
    assert(err == XMASS_SYNC_ERR_CALIBRATION_FAILED);

    /* Cleanup */
    for (int i = 0; i < XMASS_SYNC_MAX_CHANNELS; i++) {
        xmass_sync_buffer_free(&buffers[i]);
    }
    xmass_sync_deinit(&state);

    printf("PASS\n");
}

static void test_apply_corrections_with_calibration(void)
{
    printf("Test: apply_corrections with valid calibration... ");

    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;

    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);

    /* Set valid calibration with known values */
    state.calibration.valid = true;
    state.calibration.phase_offset[0] = 0.0;
    state.calibration.phase_offset[1] = M_PI / 2;  /* 90 degrees */
    state.calibration.phase_offset[2] = 0.0;
    state.calibration.phase_offset[3] = 0.0;
    state.calibration.amplitude_correction[0] = 1.0;
    state.calibration.amplitude_correction[1] = 2.0;  /* Double amplitude */
    state.calibration.amplitude_correction[2] = 1.0;
    state.calibration.amplitude_correction[3] = 1.0;

    /* Allocate and fill buffers */
    xmass_sync_buffer_t buffers[XMASS_SYNC_MAX_CHANNELS];
    for (int i = 0; i < XMASS_SYNC_MAX_CHANNELS; i++) {
        xmass_sync_buffer_alloc(&buffers[i], 10);
        for (size_t j = 0; j < 10; j++) {
            buffers[i].samples[j] = 1.0f + 0.0f * I;  /* All real, value 1 */
        }
        buffers[i].valid = true;
    }

    /* Apply corrections */
    xmass_sync_error_t err = xmass_sync_apply_corrections(&state, buffers);
    assert(err == XMASS_SYNC_OK);

    /* Channel 0: no change expected */
    assert(fabsf(crealf(buffers[0].samples[0]) - 1.0f) < 1e-5);
    assert(fabsf(cimagf(buffers[0].samples[0])) < 1e-5);

    /* Channel 1: phase rotated by -90 degrees, amplitude doubled
     * Original: 1+0i, After phase -90: 0-i (rotation), After amp *2: 0-2i */
    /* Actually: correction = amp * exp(-i * phase) = 2 * exp(-i*PI/2) = 2*(0-i) = -2i
     * So 1 * (-2i) = -2i, which means real~0, imag~-2 */
    assert(fabsf(crealf(buffers[1].samples[0])) < 1e-4);
    assert(fabsf(cimagf(buffers[1].samples[0]) + 2.0f) < 1e-4);

    /* Cleanup */
    for (int i = 0; i < XMASS_SYNC_MAX_CHANNELS; i++) {
        xmass_sync_buffer_free(&buffers[i]);
    }
    xmass_sync_deinit(&state);

    printf("PASS\n");
}

static void test_streaming_state(void)
{
    printf("Test: streaming state management... ");

    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;

    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);

    /* Initially not streaming */
    assert(state.streaming == false);

    /* Start streaming */
    xmass_sync_error_t err = xmass_sync_start_streaming(&state);
    assert(err == XMASS_SYNC_OK);
    assert(state.streaming == true);

    /* Stop streaming */
    err = xmass_sync_stop_streaming(&state);
    assert(err == XMASS_SYNC_OK);
    assert(state.streaming == false);

    /* Double stop should be safe */
    err = xmass_sync_stop_streaming(&state);
    assert(err == XMASS_SYNC_OK);

    xmass_sync_deinit(&state);

    printf("PASS\n");
}

static void test_channel_count_limit(void)
{
    printf("Test: channel count limiting... ");

    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));

    /* Set detected modules to more than max */
    carrier.num_modules_detected = 10;
    carrier.initialized = true;

    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);

    /* Should be clamped to max channels */
    assert(state.num_channels == XMASS_SYNC_MAX_CHANNELS);

    xmass_sync_deinit(&state);

    printf("PASS\n");
}

static void test_uninitialized_operations(void)
{
    printf("Test: operations on uninitialized state... ");

    xmass_sync_state_t state;
    memset(&state, 0, sizeof(state));
    state.initialized = false;

    xmass_sync_config_t config;
    xmass_sync_get_default_config(&config);

    /* Configure should fail */
    xmass_sync_error_t err = xmass_sync_configure(&state, &config);
    assert(err == XMASS_SYNC_ERR_NOT_INITIALIZED);

    /* Open devices should fail */
    err = xmass_sync_open_devices(&state);
    assert(err == XMASS_SYNC_ERR_NOT_INITIALIZED);

    /* Configure devices should fail */
    err = xmass_sync_configure_devices(&state, 100e6, 1e6, 30.0);
    assert(err == XMASS_SYNC_ERR_NOT_INITIALIZED);

    /* Setup streams should fail */
    err = xmass_sync_setup_streams(&state);
    assert(err == XMASS_SYNC_ERR_NOT_INITIALIZED);

    /* Start streaming should fail */
    err = xmass_sync_start_streaming(&state);
    assert(err == XMASS_SYNC_ERR_NOT_INITIALIZED);

    /* Calibrate should fail */
    err = xmass_sync_calibrate_phase(&state, 100e6);
    assert(err == XMASS_SYNC_ERR_NOT_INITIALIZED);

    printf("PASS\n");
}

static void test_calibration_file_error(void)
{
    printf("Test: calibration file error handling... ");

    xmass_carrier_state_t carrier;
    memset(&carrier, 0, sizeof(carrier));
    carrier.num_modules_detected = 4;
    carrier.initialized = true;

    xmass_sync_state_t state;
    xmass_sync_init(&state, &carrier);

    /* Try to load from nonexistent file */
    xmass_sync_error_t err = xmass_sync_load_calibration(&state, "/nonexistent/path/cal.txt");
    assert(err == XMASS_SYNC_ERR_DEVICE_ERROR);

    /* Try to save to invalid path */
    err = xmass_sync_save_calibration(&state, "/nonexistent/path/cal.txt");
    assert(err == XMASS_SYNC_ERR_DEVICE_ERROR);

    xmass_sync_deinit(&state);

    printf("PASS\n");
}

/*============================================================================
 * Main
 *===========================================================================*/

int main(void)
{
    printf("\n=== xMASS Synchronization Unit Tests ===\n\n");

    /* Basic functionality tests */
    test_buffer_alloc();
    test_buffer_clear();
    test_default_config();
    test_error_string();
    test_init_deinit();
    test_configure();
    test_calibration_save_load();
    test_calibration_valid();
    test_print_status();

    /* Additional comprehensive tests */
    printf("\n--- Comprehensive Tests ---\n\n");
    test_null_pointer_handling();
    test_buffer_edge_cases();
    test_calibration_get_set();
    test_apply_corrections_without_calibration();
    test_apply_corrections_with_calibration();
    test_streaming_state();
    test_channel_count_limit();
    test_uninitialized_operations();
    test_calibration_file_error();

    printf("\n=== All tests passed! ===\n\n");

    return 0;
}
