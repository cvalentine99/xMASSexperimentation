/**
 * @file xmass_sync.c
 * @brief xMASS Multi-SDR Clock Synchronization and Phase Alignment Implementation
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include "xmass_sync.h"

/*============================================================================
 * Internal Constants
 *===========================================================================*/

#define PI 3.14159265358979323846

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
    
    config->sample_rate = 1e6;              /* 1 MSPS */
    config->bandwidth = 1e6;                /* 1 MHz */
    config->gain = 40.0;                    /* 40 dB */
    config->num_samples = 8192;             /* 8K samples */
    config->calibration_freq = 100e6;       /* 100 MHz */
    config->calibration_interval = 1800;    /* 30 minutes */
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
    usleep(100000);  /* 100ms settle time */
    
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
            double phase = 2.0 * PI * j / 100.0 + (i * PI / 4.0);
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
    
    /* Configure devices for calibration */
    xmass_carrier_set_lo_frequency(state->carrier, calibration_freq);
xmass_sync_configure_devices(state, 0, 1e6, 40.0);
    xmass_sync_setup_streams(state);
    xmass_sync_start_streaming(state);
    
    /* Capture multiple snapshots for averaging */
    double phase_measurements[XMASS_SYNC_CAL_SNAPSHOTS][XMASS_SYNC_MAX_CHANNELS];
    double amplitude_measurements[XMASS_SYNC_CAL_SNAPSHOTS][XMASS_SYNC_MAX_CHANNELS];
    
    printf("[xMASS Sync] Capturing %d snapshots for calibration...\n",
           XMASS_SYNC_CAL_SNAPSHOTS);
    
    for (int snapshot = 0; snapshot < XMASS_SYNC_CAL_SNAPSHOTS; snapshot++) {
        /* Capture samples */
        xmass_sync_capture_samples(state, buffers, XMASS_SYNC_DEFAULT_CAL_SAMPLES);
        
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
            }
        }
        
        usleep(10000);  /* 10ms between snapshots */
    }
    
    xmass_sync_stop_streaming(state);
    
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
        if (ch == 0) {
            state->calibration.amplitude_correction[ch] = 1.0;
        } else {
            double ref_amp = 0.0;
            for (int s = 0; s < XMASS_SYNC_CAL_SNAPSHOTS; s++) {
                ref_amp += amplitude_measurements[s][0];
            }
            ref_amp /= XMASS_SYNC_CAL_SNAPSHOTS;
            state->calibration.amplitude_correction[ch] = ref_amp / avg_amp;
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
               state->calibration.phase_offset[ch] * 180.0 / PI,
               state->calibration.amplitude_correction[ch],
               state->calibration.phase_stability[ch] * 180.0 / PI);
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
    if (!state || !filename) {
        return XMASS_SYNC_ERR_INVALID_PARAM;
    }
    
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        return XMASS_SYNC_ERR_DEVICE_ERROR;
    }
    
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        
        char key[64];
        double value;
        
        if (sscanf(line, "%63[^=]=%lf", key, &value) == 2) {
            if (strcmp(key, "calibration_freq") == 0) {
                state->calibration.calibration_freq = value;
            } else if (strcmp(key, "timestamp") == 0) {
                state->calibration.calibration_timestamp = (uint64_t)value;
            } else if (strcmp(key, "valid") == 0) {
                state->calibration.valid = (value != 0);
            } else if (strcmp(key, "count") == 0) {
                state->calibration.calibration_count = (uint32_t)value;
            } else {
                /* Parse channel-specific values */
                int ch;
                char param[32];
                if (sscanf(key, "ch%d_%31s", &ch, param) == 2) {
                    if (ch >= 0 && ch < XMASS_SYNC_MAX_CHANNELS) {
                        if (strcmp(param, "phase") == 0) {
                            state->calibration.phase_offset[ch] = value;
                        } else if (strcmp(param, "amplitude") == 0) {
                            state->calibration.amplitude_correction[ch] = value;
                        } else if (strcmp(param, "stability") == 0) {
                            state->calibration.phase_stability[ch] = value;
                        }
                    }
                }
            }
        }
    }
    
    fclose(fp);
    
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
        xmass_sync_error_t err = xmass_sync_buffer_alloc(&buffers[i], 4096);
        if (err != XMASS_SYNC_OK) {
            for (int j = 0; j < i; j++) {
                xmass_sync_buffer_free(&buffers[j]);
            }
            return err;
        }
    }
    
    /* Capture samples */
    xmass_sync_capture_samples(state, buffers, 4096);
    
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
                   ch, phase_diffs[ch] * 180.0 / PI, max_phase_error * 180.0 / PI);
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
                   state->calibration.phase_offset[ch] * 180.0 / PI,
                   state->calibration.amplitude_correction[ch],
                   state->calibration.phase_stability[ch] * 180.0 / PI);
        }
    }
    
    printf("\n");
}
