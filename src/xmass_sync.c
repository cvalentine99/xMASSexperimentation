/**
 * @file xmass_sync.c
 * @brief xMASS Multi-SDR Clock Synchronization and Phase Alignment Implementation
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#define _USE_MATH_DEFINES  /* For M_PI on some systems */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include "xmass_sync.h"

/* Ensure M_PI is defined */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*============================================================================
 * Error Strings
 *===========================================================================*/

static const char *error_strings[] = {
    "Success",
    "Invalid parameter",
    "Not initialized",
    "Device error",
    "Calibration failed",
    "No signal detected",
    "Timeout",
    "Memory allocation failed"
};

const char* xmass_sync_error_string(xmass_sync_error_t error)
{
    int idx = -error;
    if (idx >= 0 && idx < (int)(sizeof(error_strings)/sizeof(error_strings[0]))) {
        return error_strings[idx];
    }
    return "Unknown error";
}

/*============================================================================
 * Buffer Management
 *===========================================================================*/

xmass_sync_error_t xmass_sync_buffer_alloc(xmass_sync_buffer_t *buffer,
                                            size_t num_samples)
{
    if (!buffer || num_samples == 0) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    buffer->samples = (float complex *)calloc(num_samples, sizeof(float complex));
    if (!buffer->samples) {
        return XMASS_SYNC_ERR_MEMORY;
    }
    
    buffer->num_samples = num_samples;
    buffer->timestamp = 0;
    buffer->channel = 0;
    buffer->valid = false;
    
    return XMASS_SYNC_OK;
}

void xmass_sync_buffer_free(xmass_sync_buffer_t *buffer)
{
    if (buffer && buffer->samples) {
        free(buffer->samples);
        buffer->samples = NULL;
        buffer->num_samples = 0;
        buffer->valid = false;
    }
}

void xmass_sync_buffer_clear(xmass_sync_buffer_t *buffer)
{
    if (buffer && buffer->samples) {
        memset(buffer->samples, 0, buffer->num_samples * sizeof(float complex));
        buffer->valid = false;
    }
}

/*============================================================================
 * Initialization and Configuration
 *===========================================================================*/

void xmass_sync_get_default_config(xmass_sync_config_t *config)
{
    if (!config) return;

    config->sample_rate = XMASS_SYNC_DEFAULT_SAMPLE_RATE;
    config->bandwidth = XMASS_SYNC_DEFAULT_BANDWIDTH;
    config->gain = XMASS_SYNC_DEFAULT_GAIN;
    config->num_samples = XMASS_SYNC_DEFAULT_CAL_SAMPLES;
    config->calibration_freq = XMASS_SYNC_DEFAULT_CAL_FREQ;
    config->calibration_interval = XMASS_SYNC_DEFAULT_CAL_INTERVAL;
    config->auto_recalibrate = false;
}

xmass_sync_error_t xmass_sync_init(xmass_sync_state_t *state,
                                    xmass_carrier_state_t *carrier)
{
    if (!state || !carrier) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    /* Clear state */
    memset(state, 0, sizeof(xmass_sync_state_t));
    
    /* Store carrier reference */
    state->carrier = carrier;
    
    /* Set default configuration */
    xmass_sync_get_default_config(&state->config);
    
    /* Initialize calibration */
    for (int i = 0; i < XMASS_SYNC_MAX_CHANNELS; i++) {
        state->calibration.phase_offset[i] = 0.0;
        state->calibration.amplitude_correction[i] = 1.0;
        state->calibration.phase_stability[i] = 0.0;
    }
    state->calibration.valid = false;
    
    /* Count available channels */
    state->num_channels = carrier->num_modules_detected;
    if (state->num_channels > XMASS_SYNC_MAX_CHANNELS) {
        state->num_channels = XMASS_SYNC_MAX_CHANNELS;
    }
    
    state->initialized = true;
    
    printf("[xMASS Sync] Initialized with %d channels\n", state->num_channels);
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_deinit(xmass_sync_state_t *state)
{
    if (!state) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    /* Stop streaming if active */
    if (state->streaming) {
        xmass_sync_stop_streaming(state);
    }
    
    /* Close devices */
    xmass_sync_close_devices(state);
    
    state->initialized = false;
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_configure(xmass_sync_state_t *state,
                                         const xmass_sync_config_t *config)
{
    if (!state || !config) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    if (!state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    memcpy(&state->config, config, sizeof(xmass_sync_config_t));
    
    return XMASS_SYNC_OK;
}

/*============================================================================
 * Device Management (SoapySDR Integration)
 *===========================================================================*/

/*
 * Note: This implementation uses placeholder code for SoapySDR integration.
 * In production, this would use the actual SoapySDR API:
 * 
 * #include <SoapySDR/Device.h>
 * #include <SoapySDR/Formats.h>
 */

xmass_sync_error_t xmass_sync_open_devices(xmass_sync_state_t *state)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    printf("[xMASS Sync] Opening %d xSDR devices...\n", state->num_channels);
    
    /*
     * SoapySDR implementation would be:
     * 
     * SoapySDRKwargs args = {0};
     * SoapySDRKwargs_set(&args, "driver", "usdr");
     * 
     * for (int i = 0; i < state->num_channels; i++) {
     *     char index_str[8];
     *     snprintf(index_str, sizeof(index_str), "%d", i);
     *     SoapySDRKwargs_set(&args, "index", index_str);
     *     
     *     state->devices[i] = SoapySDRDevice_make(&args);
     *     if (!state->devices[i]) {
     *         printf("[xMASS Sync] Failed to open device %d\n", i);
     *         return XMASS_SYNC_ERR_DEVICE_ERROR;
     *     }
     *     printf("[xMASS Sync] Opened usdr%d\n", i);
     * }
     */
    
    for (int i = 0; i < state->num_channels; i++) {
        /* Placeholder - would open actual device */
        state->devices[i] = (void *)(intptr_t)(i + 1);  /* Non-NULL placeholder */
        printf("[xMASS Sync] Opened usdr%d\n", i);
    }
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_close_devices(xmass_sync_state_t *state)
{
    if (!state) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    for (int i = 0; i < state->num_channels; i++) {
        if (state->devices[i]) {
            /*
             * SoapySDR: SoapySDRDevice_unmake(state->devices[i]);
             */
            state->devices[i] = NULL;
        }
    }
    
    printf("[xMASS Sync] All devices closed\n");
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_configure_devices(xmass_sync_state_t *state,
                                                 double frequency,
                                                 double sample_rate,
                                                 double gain)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    printf("[xMASS Sync] Configuring all devices:\n");
    printf("  Frequency: %.3f MHz\n", frequency / 1e6);
    printf("  Sample Rate: %.3f MSPS\n", sample_rate / 1e6);
    printf("  Gain: %.1f dB\n", gain);
    
    for (int i = 0; i < state->num_channels; i++) {
        if (!state->devices[i]) continue;
        
        /*
         * SoapySDR implementation:
         * 
         * SoapySDRDevice_setFrequency(state->devices[i], SOAPY_SDR_RX, 0, 
         *                             frequency, NULL);
         * SoapySDRDevice_setSampleRate(state->devices[i], SOAPY_SDR_RX, 0,
         *                              sample_rate);
         * SoapySDRDevice_setBandwidth(state->devices[i], SOAPY_SDR_RX, 0,
         *                             sample_rate);
         * SoapySDRDevice_setGain(state->devices[i], SOAPY_SDR_RX, 0, gain);
         */
        
        printf("  usdr%d: configured\n", i);
    }
    
    state->config.sample_rate = sample_rate;
    state->config.gain = gain;
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_verify_ref_clock(xmass_sync_state_t *state)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    printf("[xMASS Sync] Verifying reference clock on all devices...\n");
    
    for (int i = 0; i < state->num_channels; i++) {
        double freq_hz;
        xmass_error_t err = xmass_carrier_get_ref_clock(state->carrier, i, &freq_hz);
        
        if (err == XMASS_OK) {
            printf("  usdr%d: %.2f MHz\n", i, freq_hz / 1e6);
        } else {
            printf("  usdr%d: unable to read\n", i);
        }
    }
    
    printf("[xMASS Sync] Note: LMK05318B configuration is pre-set by carrier board\n");
    
    return XMASS_SYNC_OK;
}

/*============================================================================
 * Streaming Control
 *===========================================================================*/

xmass_sync_error_t xmass_sync_setup_streams(xmass_sync_state_t *state)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    printf("[xMASS Sync] Setting up RX streams...\n");
    
    for (int i = 0; i < state->num_channels; i++) {
        if (!state->devices[i]) continue;
        
        /*
         * SoapySDR implementation:
         * 
         * state->streams[i] = SoapySDRDevice_setupStream(
         *     state->devices[i], SOAPY_SDR_RX, SOAPY_SDR_CF32, NULL, 0, NULL);
         */
        
        state->streams[i] = (void *)(intptr_t)(i + 1);  /* Placeholder */
        printf("  usdr%d: stream configured\n", i);
    }
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_start_streaming(xmass_sync_state_t *state)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    printf("[xMASS Sync] Starting streaming on all devices...\n");
    
    for (int i = 0; i < state->num_channels; i++) {
        if (!state->devices[i] || !state->streams[i]) continue;
        
        /*
         * SoapySDR: SoapySDRDevice_activateStream(state->devices[i],
         *                                         state->streams[i], 0, 0, 0);
         */
    }
    
    state->streaming = true;
    usleep(XMASS_SYNC_STREAM_SETTLE_MS * 1000);  /* Convert ms to us */

    printf("[xMASS Sync] All devices streaming\n");
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_stop_streaming(xmass_sync_state_t *state)
{
    if (!state) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    printf("[xMASS Sync] Stopping streaming...\n");
    
    for (int i = 0; i < state->num_channels; i++) {
        if (!state->devices[i] || !state->streams[i]) continue;
        
        /*
         * SoapySDR: SoapySDRDevice_deactivateStream(state->devices[i],
         *                                           state->streams[i], 0, 0);
         */
    }
    
    state->streaming = false;
    
    printf("[xMASS Sync] Streaming stopped\n");
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_capture_samples(xmass_sync_state_t *state,
                                               xmass_sync_buffer_t *buffers,
                                               size_t num_samples)
{
    if (!state || !buffers || !state->initialized) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    for (int i = 0; i < state->num_channels; i++) {
        if (!state->devices[i] || !state->streams[i]) continue;
        
        /*
         * SoapySDR implementation:
         * 
         * void *buf_ptrs[1] = {buffers[i].samples};
         * int flags = 0;
         * long long time_ns = 0;
         * 
         * int ret = SoapySDRDevice_readStream(state->devices[i],
         *                                     state->streams[i],
         *                                     buf_ptrs, num_samples,
         *                                     &flags, &time_ns, 1000000);
         * 
         * if (ret > 0) {
         *     buffers[i].num_samples = ret;
         *     buffers[i].timestamp = time_ns;
         *     buffers[i].valid = true;
         * }
         */
        
        /* Placeholder: generate test data */
        for (size_t j = 0; j < num_samples && j < buffers[i].num_samples; j++) {
            double phase = 2.0 * M_PI * j / 100.0 + (i * M_PI / 4.0);
            buffers[i].samples[j] = cosf(phase) + I * sinf(phase);
        }
        buffers[i].num_samples = num_samples;
        buffers[i].channel = i;
        buffers[i].valid = true;
    }
    
    state->samples_captured += num_samples;
    
    return XMASS_SYNC_OK;
}

/*============================================================================
 * Phase Calibration
 *===========================================================================*/

xmass_sync_error_t xmass_sync_calibrate_phase(xmass_sync_state_t *state,
                                               double calibration_freq)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    printf("[xMASS Sync] Starting phase calibration at %.1f MHz...\n",
           calibration_freq / 1e6);
    printf("  Ensure a strong CW signal is present at this frequency!\n");
    
    /* Allocate calibration buffers */
    xmass_sync_buffer_t buffers[XMASS_SYNC_MAX_CHANNELS];
    for (int i = 0; i < state->num_channels; i++) {
        xmass_sync_error_t err = xmass_sync_buffer_alloc(&buffers[i],
                                                          XMASS_SYNC_DEFAULT_CAL_SAMPLES);
        if (err != XMASS_SYNC_OK) {
            /* Cleanup and return */
            for (int j = 0; j < i; j++) {
                xmass_sync_buffer_free(&buffers[j]);
            }
            return err;
        }
    }
    
    /* Configure devices for calibration
     * Note: LO is set to calibration_freq, SDR tuned to baseband (0 Hz)
     * since the LMK05318B distributes the common LO to all modules
     */
    xmass_error_t carrier_err = xmass_carrier_set_lo_frequency(state->carrier, calibration_freq);
    if (carrier_err != XMASS_OK && carrier_err != XMASS_ERR_LMK_NOT_ACCESSIBLE) {
        printf("[xMASS Sync] Warning: Failed to set LO frequency (error %d)\n", carrier_err);
    }

    xmass_sync_error_t dev_err = xmass_sync_configure_devices(state, 0, 1e6, 40.0);
    if (dev_err != XMASS_SYNC_OK) {
        printf("[xMASS Sync] Warning: Failed to configure devices (error %d)\n", dev_err);
    }

    xmass_sync_error_t stream_err = xmass_sync_setup_streams(state);
    if (stream_err != XMASS_SYNC_OK) {
        printf("[xMASS Sync] Warning: Failed to setup streams (error %d)\n", stream_err);
    }

    xmass_sync_error_t start_err = xmass_sync_start_streaming(state);
    if (start_err != XMASS_SYNC_OK) {
        printf("[xMASS Sync] Warning: Failed to start streaming (error %d)\n", start_err);
    }
    
    /* Capture multiple snapshots for averaging */
    double phase_measurements[XMASS_SYNC_CAL_SNAPSHOTS][XMASS_SYNC_MAX_CHANNELS];
    double amplitude_measurements[XMASS_SYNC_CAL_SNAPSHOTS][XMASS_SYNC_MAX_CHANNELS];
    
    printf("[xMASS Sync] Capturing %d snapshots for calibration...\n",
           XMASS_SYNC_CAL_SNAPSHOTS);

    int failed_captures = 0;
    double min_signal_level = 1e10;

    for (int snapshot = 0; snapshot < XMASS_SYNC_CAL_SNAPSHOTS; snapshot++) {
        /* Capture samples with error checking */
        xmass_sync_error_t cap_err = xmass_sync_capture_samples(state, buffers, XMASS_SYNC_DEFAULT_CAL_SAMPLES);
        if (cap_err != XMASS_SYNC_OK) {
            printf("[xMASS Sync] Warning: Capture failed on snapshot %d (error %d)\n", snapshot, cap_err);
            failed_captures++;
            if (failed_captures > XMASS_SYNC_CAL_SNAPSHOTS / 2) {
                /* Too many failures, abort calibration */
                xmass_sync_stop_streaming(state);
                for (int i = 0; i < state->num_channels; i++) {
                    xmass_sync_buffer_free(&buffers[i]);
                }
                return XMASS_SYNC_ERR_CALIBRATION_FAILED;
            }
            continue;
        }

        /* Verify buffers are valid */
        bool valid_capture = true;
        for (int ch = 0; ch < state->num_channels; ch++) {
            if (!buffers[ch].valid || buffers[ch].num_samples == 0) {
                valid_capture = false;
                break;
            }
        }
        if (!valid_capture) {
            failed_captures++;
            continue;
        }

        /* Measure phase and amplitude relative to channel 0 */
        for (int ch = 0; ch < state->num_channels; ch++) {
            if (ch == 0) {
                phase_measurements[snapshot][0] = 0.0;

                /* Calculate mean amplitude */
                double sum = 0.0;
                for (size_t j = 0; j < buffers[0].num_samples; j++) {
                    sum += cabsf(buffers[0].samples[j]);
                }
                amplitude_measurements[snapshot][0] = sum / buffers[0].num_samples;

                /* Track minimum signal level for quality check */
                if (amplitude_measurements[snapshot][0] < min_signal_level) {
                    min_signal_level = amplitude_measurements[snapshot][0];
                }
            } else {
                /* Cross-correlation with reference (channel 0) */
                float complex correlation = 0.0f;
                for (size_t j = 0; j < buffers[ch].num_samples; j++) {
                    correlation += buffers[ch].samples[j] * conjf(buffers[0].samples[j]);
                }
                correlation /= buffers[ch].num_samples;

                phase_measurements[snapshot][ch] = cargf(correlation);

                /* Calculate mean amplitude */
                double sum = 0.0;
                for (size_t j = 0; j < buffers[ch].num_samples; j++) {
                    sum += cabsf(buffers[ch].samples[j]);
                }
                amplitude_measurements[snapshot][ch] = sum / buffers[ch].num_samples;

                if (amplitude_measurements[snapshot][ch] < min_signal_level) {
                    min_signal_level = amplitude_measurements[snapshot][ch];
                }
            }
        }

        usleep(XMASS_SYNC_SNAPSHOT_DELAY_MS * 1000);  /* Convert ms to us */
    }

    xmass_sync_stop_streaming(state);

    /* Check if we got enough successful captures */
    int successful_captures = XMASS_SYNC_CAL_SNAPSHOTS - failed_captures;
    if (successful_captures < XMASS_SYNC_CAL_SNAPSHOTS / 2) {
        printf("[xMASS Sync] Error: Only %d/%d successful captures\n",
               successful_captures, XMASS_SYNC_CAL_SNAPSHOTS);
        for (int i = 0; i < state->num_channels; i++) {
            xmass_sync_buffer_free(&buffers[i]);
        }
        return XMASS_SYNC_ERR_CALIBRATION_FAILED;
    }

    /* Check signal level */
    if (min_signal_level < XMASS_SYNC_MIN_AMPLITUDE) {
        printf("[xMASS Sync] Warning: Very weak signal detected (level=%.2e)\n", min_signal_level);
        printf("[xMASS Sync] Calibration may be unreliable. Check input signal.\n");
        /* Continue anyway but warn */
    }
    
    /* Average measurements */
    for (int ch = 0; ch < state->num_channels; ch++) {
        double phase_sum = 0.0;
        double amp_sum = 0.0;
        
        for (int s = 0; s < XMASS_SYNC_CAL_SNAPSHOTS; s++) {
            phase_sum += phase_measurements[s][ch];
            amp_sum += amplitude_measurements[s][ch];
        }
        
        state->calibration.phase_offset[ch] = phase_sum / XMASS_SYNC_CAL_SNAPSHOTS;
        double avg_amp = amp_sum / XMASS_SYNC_CAL_SNAPSHOTS;
        
        /* Amplitude correction relative to channel 0 */
        /* Protect against division by zero for dead channels */
        if (ch == 0) {
            state->calibration.amplitude_correction[ch] = 1.0;
        } else {
            double ref_amp = 0.0;
            for (int s = 0; s < XMASS_SYNC_CAL_SNAPSHOTS; s++) {
                ref_amp += amplitude_measurements[s][0];
            }
            ref_amp /= XMASS_SYNC_CAL_SNAPSHOTS;
            double safe_avg_amp = (avg_amp > XMASS_SYNC_MIN_AMPLITUDE) ? avg_amp : XMASS_SYNC_MIN_AMPLITUDE;
            state->calibration.amplitude_correction[ch] = ref_amp / safe_avg_amp;
            if (avg_amp < XMASS_SYNC_MIN_AMPLITUDE) {
                printf("[xMASS Sync] Warning: Channel %d has near-zero amplitude\n", ch);
            }
        }
        
        /* Calculate phase stability (std dev) */
        double phase_var = 0.0;
        for (int s = 0; s < XMASS_SYNC_CAL_SNAPSHOTS; s++) {
            double diff = phase_measurements[s][ch] - state->calibration.phase_offset[ch];
            phase_var += diff * diff;
        }
        state->calibration.phase_stability[ch] = sqrt(phase_var / XMASS_SYNC_CAL_SNAPSHOTS);
    }
    
    /* Update calibration metadata */
    state->calibration.calibration_freq = calibration_freq;
    state->calibration.calibration_timestamp = (uint64_t)time(NULL);
    state->calibration.valid = true;
    state->calibration.calibration_count++;
    
    /* Print results */
    printf("\n[xMASS Sync] Calibration complete:\n");
    printf("  Channel | Phase Offset | Amplitude Correction | Stability\n");
    printf("  --------|--------------|----------------------|----------\n");
    for (int ch = 0; ch < state->num_channels; ch++) {
        printf("  usdr%d   | %7.2f°    | %6.3fx              | ±%.2f°\n",
               ch,
               state->calibration.phase_offset[ch] * 180.0 / M_PI,
               state->calibration.amplitude_correction[ch],
               state->calibration.phase_stability[ch] * 180.0 / M_PI);
    }
    
    /* Cleanup */
    for (int i = 0; i < state->num_channels; i++) {
        xmass_sync_buffer_free(&buffers[i]);
    }
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_apply_corrections(xmass_sync_state_t *state,
                                                 xmass_sync_buffer_t *buffers)
{
    if (!state || !buffers || !state->initialized) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    if (!state->calibration.valid) {
        return XMASS_SYNC_ERR_CALIBRATION_FAILED;
    }
    
    for (int ch = 0; ch < state->num_channels; ch++) {
        if (!buffers[ch].valid) continue;
        
        /* Apply amplitude correction */
        float amp_corr = (float)state->calibration.amplitude_correction[ch];
        
        /* Apply phase correction (rotate by negative of measured offset) */
        float complex phase_corr = cexpf(-I * (float)state->calibration.phase_offset[ch]);
        
        for (size_t j = 0; j < buffers[ch].num_samples; j++) {
            buffers[ch].samples[j] *= amp_corr * phase_corr;
        }
    }
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_get_calibration(xmass_sync_state_t *state,
                                               xmass_sync_calibration_t *calibration)
{
    if (!state || !calibration) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    memcpy(calibration, &state->calibration, sizeof(xmass_sync_calibration_t));
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_set_calibration(xmass_sync_state_t *state,
                                               const xmass_sync_calibration_t *calibration)
{
    if (!state || !calibration) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    memcpy(&state->calibration, calibration, sizeof(xmass_sync_calibration_t));
    
    return XMASS_SYNC_OK;
}

bool xmass_sync_calibration_valid(xmass_sync_state_t *state)
{
    if (!state || !state->calibration.valid) {
        return false;
    }
    
    /* Check if calibration has expired */
    uint64_t now = (uint64_t)time(NULL);
    uint64_t age = now - state->calibration.calibration_timestamp;
    
    if (age > state->config.calibration_interval) {
        return false;
    }
    
    return true;
}

xmass_sync_error_t xmass_sync_save_calibration(xmass_sync_state_t *state,
                                                const char *filename)
{
    if (!state || !filename) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        return XMASS_SYNC_ERR_DEVICE_ERROR;
    }
    
    fprintf(fp, "# xMASS Calibration Data\n");
    fprintf(fp, "# Generated: %lu\n", (unsigned long)state->calibration.calibration_timestamp);
    fprintf(fp, "# Calibration Frequency: %.0f Hz\n", state->calibration.calibration_freq);
    fprintf(fp, "\n");
    
    fprintf(fp, "calibration_freq=%.0f\n", state->calibration.calibration_freq);
    fprintf(fp, "timestamp=%lu\n", (unsigned long)state->calibration.calibration_timestamp);
    fprintf(fp, "valid=%d\n", state->calibration.valid ? 1 : 0);
    fprintf(fp, "count=%u\n", state->calibration.calibration_count);
    
    for (int ch = 0; ch < XMASS_SYNC_MAX_CHANNELS; ch++) {
        fprintf(fp, "ch%d_phase=%.9f\n", ch, state->calibration.phase_offset[ch]);
        fprintf(fp, "ch%d_amplitude=%.9f\n", ch, state->calibration.amplitude_correction[ch]);
        fprintf(fp, "ch%d_stability=%.9f\n", ch, state->calibration.phase_stability[ch]);
    }
    
    fclose(fp);
    
    printf("[xMASS Sync] Calibration saved to %s\n", filename);
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_load_calibration(xmass_sync_state_t *state,
                                                const char *filename)
{
    if (!state || !filename || filename[0] == '\0') {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }

    FILE *fp = fopen(filename, "r");
    if (!fp) {
        return XMASS_SYNC_ERR_DEVICE_ERROR;
    }

    /* Initialize calibration to default/invalid state before loading */
    memset(&state->calibration, 0, sizeof(state->calibration));

    char line[256];
    int valid_entries = 0;
    int phase_entries = 0;
    int amplitude_entries = 0;

    while (fgets(line, sizeof(line), fp)) {
        /* Check for line truncation */
        size_t len = strlen(line);
        if (len > 0 && line[len - 1] != '\n' && !feof(fp)) {
            /* Skip truncated line */
            int c;
            while ((c = fgetc(fp)) != '\n' && c != EOF);
            continue;
        }

        /* Remove trailing whitespace */
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' ||
                          line[len - 1] == ' ' || line[len - 1] == '\t')) {
            line[--len] = '\0';
        }

        /* Skip empty lines and comments */
        if (len == 0 || line[0] == '#') continue;

        char key[64];
        double value;

        if (sscanf(line, "%63[^=]=%lf", key, &value) != 2) {
            continue;  /* Skip malformed lines */
        }

        /* Validate and store values */
        if (strcmp(key, "calibration_freq") == 0) {
            /* Frequency should be positive and reasonable (1 MHz to 10 GHz) */
            if (value >= 1e6 && value <= 10e9) {
                state->calibration.calibration_freq = value;
                valid_entries++;
            }
        } else if (strcmp(key, "timestamp") == 0) {
            /* Timestamp should be a positive number */
            if (value >= 0) {
                state->calibration.calibration_timestamp = (uint64_t)value;
                valid_entries++;
            }
        } else if (strcmp(key, "valid") == 0) {
            state->calibration.valid = (value != 0);
            valid_entries++;
        } else if (strcmp(key, "count") == 0) {
            if (value >= 0 && value <= UINT32_MAX) {
                state->calibration.calibration_count = (uint32_t)value;
                valid_entries++;
            }
        } else {
            /* Parse channel-specific values */
            int ch;
            char param[32];
            if (sscanf(key, "ch%d_%31s", &ch, param) == 2) {
                if (ch >= 0 && ch < XMASS_SYNC_MAX_CHANNELS) {
                    if (strcmp(param, "phase") == 0) {
                        /* Phase should be in radians, typically -2*pi to 2*pi */
                        if (value >= -2 * M_PI && value <= 2 * M_PI) {
                            state->calibration.phase_offset[ch] = value;
                            phase_entries++;
                        }
                    } else if (strcmp(param, "amplitude") == 0) {
                        /* Amplitude correction should be positive */
                        if (value > 0 && value < 100.0) {
                            state->calibration.amplitude_correction[ch] = value;
                            amplitude_entries++;
                        }
                    } else if (strcmp(param, "stability") == 0) {
                        /* Stability (std dev) should be non-negative */
                        if (value >= 0 && value <= M_PI) {
                            state->calibration.phase_stability[ch] = value;
                        }
                    }
                }
            }
        }
    }

    fclose(fp);

    /* Validate that we got a complete calibration */
    if (valid_entries < 3 || phase_entries < XMASS_SYNC_MAX_CHANNELS ||
        amplitude_entries < XMASS_SYNC_MAX_CHANNELS) {
        printf("[xMASS Sync] Warning: Incomplete calibration file (valid=%d, phase=%d, amp=%d)\n",
               valid_entries, phase_entries, amplitude_entries);
        state->calibration.valid = false;
        return XMASS_SYNC_ERR_CALIBRATION_FAILED;
    }

    printf("[xMASS Sync] Calibration loaded from %s\n", filename);

    return XMASS_SYNC_OK;
}

/*============================================================================
 * Hardware Synchronization
 *===========================================================================*/

xmass_sync_error_t xmass_sync_hardware_sync(xmass_sync_state_t *state)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    if (!xmass_carrier_lmk_accessible(state->carrier)) {
        printf("[xMASS Sync] Hardware SYNC not available (LMK05318B not accessible)\n");
        return XMASS_SYNC_ERR_DEVICE_ERROR;
    }
    
    xmass_error_t err = xmass_carrier_sync_clocks(state->carrier);
    if (err != XMASS_OK) {
        return XMASS_SYNC_ERR_DEVICE_ERROR;
    }
    
    state->sync_events++;
    
    printf("[xMASS Sync] Hardware SYNC triggered\n");
    
    return XMASS_SYNC_OK;
}

xmass_sync_error_t xmass_sync_configure_lmk(xmass_sync_state_t *state,
                                             double output_freq)
{
    if (!state || !state->initialized) {
        return XMASS_SYNC_ERR_NOT_INITIALIZED;
    }
    
    if (!xmass_carrier_lmk_accessible(state->carrier)) {
        printf("[xMASS Sync] LMK05318B not accessible\n");
        return XMASS_SYNC_ERR_DEVICE_ERROR;
    }
    
    xmass_error_t err = xmass_carrier_set_ref_clock(state->carrier, output_freq);
    if (err != XMASS_OK) {
        return XMASS_SYNC_ERR_DEVICE_ERROR;
    }
    
    printf("[xMASS Sync] LMK05318B configured for %.2f MHz output\n",
           output_freq / 1e6);
    
    return XMASS_SYNC_OK;
}

/*============================================================================
 * Coherence Verification
 *===========================================================================*/

xmass_sync_error_t xmass_sync_measure_coherence(xmass_sync_state_t *state,
                                                 double *phase_diffs)
{
    if (!state || !phase_diffs || !state->initialized) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    /* Allocate buffers */
    xmass_sync_buffer_t buffers[XMASS_SYNC_MAX_CHANNELS];
    for (int i = 0; i < state->num_channels; i++) {
        xmass_sync_error_t err = xmass_sync_buffer_alloc(&buffers[i], XMASS_SYNC_COHERENCE_SAMPLES);
        if (err != XMASS_SYNC_OK) {
            for (int j = 0; j < i; j++) {
                xmass_sync_buffer_free(&buffers[j]);
            }
            return err;
        }
    }

    /* Capture samples */
    xmass_sync_capture_samples(state, buffers, XMASS_SYNC_COHERENCE_SAMPLES);
    
    /* Apply corrections */
    xmass_sync_apply_corrections(state, buffers);
    
    /* Measure phase differences */
    for (int ch = 1; ch < state->num_channels; ch++) {
        float complex correlation = 0.0f;
        for (size_t j = 0; j < buffers[ch].num_samples; j++) {
            correlation += buffers[ch].samples[j] * conjf(buffers[0].samples[j]);
        }
        phase_diffs[ch] = cargf(correlation);
    }
    phase_diffs[0] = 0.0;
    
    /* Cleanup */
    for (int i = 0; i < state->num_channels; i++) {
        xmass_sync_buffer_free(&buffers[i]);
    }
    
    return XMASS_SYNC_OK;
}

bool xmass_sync_verify_quality(xmass_sync_state_t *state, double max_phase_error)
{
    if (!state || !state->initialized) {
        return false;
    }
    
    double phase_diffs[XMASS_SYNC_MAX_CHANNELS];
    if (xmass_sync_measure_coherence(state, phase_diffs) != XMASS_SYNC_OK) {
        return false;
    }
    
    for (int ch = 1; ch < state->num_channels; ch++) {
        if (fabs(phase_diffs[ch]) > max_phase_error) {
            printf("[xMASS Sync] Channel %d phase error %.2f° exceeds limit %.2f°\n",
                   ch, phase_diffs[ch] * 180.0 / M_PI, max_phase_error * 180.0 / M_PI);
            return false;
        }
    }
    
    return true;
}

/*============================================================================
 * Diagnostics
 *===========================================================================*/

void xmass_sync_print_status(xmass_sync_state_t *state)
{
    if (!state) {
        printf("[xMASS Sync] State is NULL\n");
        return;
    }
    
    printf("\n=== xMASS Synchronization Status ===\n\n");
    
    printf("Initialized: %s\n", state->initialized ? "Yes" : "No");
    printf("Streaming: %s\n", state->streaming ? "Yes" : "No");
    printf("Channels: %d\n", state->num_channels);
    printf("Samples captured: %lu\n", (unsigned long)state->samples_captured);
    printf("Sync events: %lu\n", (unsigned long)state->sync_events);
    
    printf("\nConfiguration:\n");
    printf("  Sample rate: %.3f MSPS\n", state->config.sample_rate / 1e6);
    printf("  Bandwidth: %.3f MHz\n", state->config.bandwidth / 1e6);
    printf("  Gain: %.1f dB\n", state->config.gain);
    printf("  Calibration freq: %.1f MHz\n", state->config.calibration_freq / 1e6);
    printf("  Calibration interval: %u seconds\n", state->config.calibration_interval);
    
    printf("\nCalibration:\n");
    printf("  Valid: %s\n", state->calibration.valid ? "Yes" : "No");
    if (state->calibration.valid) {
        printf("  Frequency: %.1f MHz\n", state->calibration.calibration_freq / 1e6);
        printf("  Age: %lu seconds\n",
               (unsigned long)(time(NULL) - state->calibration.calibration_timestamp));
        printf("  Count: %u\n", state->calibration.calibration_count);
        
        printf("\n  Channel | Phase Offset | Amplitude | Stability\n");
        printf("  --------|--------------|-----------|----------\n");
        for (int ch = 0; ch < state->num_channels; ch++) {
            printf("  %d       | %7.2f°    | %6.3fx   | ±%.2f°\n",
                   ch,
                   state->calibration.phase_offset[ch] * 180.0 / M_PI,
                   state->calibration.amplitude_correction[ch],
                   state->calibration.phase_stability[ch] * 180.0 / M_PI);
        }
    }
    
    printf("\n");
}
