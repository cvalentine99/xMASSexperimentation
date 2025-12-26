/**
 * @file xmass_sync.h
 * @brief xMASS Multi-SDR Clock Synchronization and Phase Alignment
 * 
 * Provides synchronization capabilities for coherent direction finding
 * using 4 xSDR modules on the xMASS carrier board.
 * 
 * @copyright 2025 Wavelet Lab
 * @license MIT
 */

#ifndef XMASS_SYNC_H
#define XMASS_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include <complex.h>
#include "xmass_carrier.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Constants
 *===========================================================================*/

#define XMASS_SYNC_MAX_CHANNELS     4       /* Maximum number of channels */
#define XMASS_SYNC_DEFAULT_CAL_FREQ 100e6   /* Default calibration frequency */
#define XMASS_SYNC_DEFAULT_CAL_SAMPLES 8192 /* Default calibration samples */
#define XMASS_SYNC_CAL_SNAPSHOTS    10      /* Number of snapshots for averaging */

/* Timing constants (milliseconds) */
#define XMASS_SYNC_STREAM_SETTLE_MS     100     /* Time to wait after starting streams */
#define XMASS_SYNC_SNAPSHOT_DELAY_MS    10      /* Delay between calibration snapshots */

/* Buffer sizes */
#define XMASS_SYNC_COHERENCE_SAMPLES    4096    /* Samples for coherence measurement */

/* Calibration quality thresholds */
#define XMASS_SYNC_MIN_AMPLITUDE        1e-10   /* Minimum amplitude for valid signal */

/* Default configuration values */
#define XMASS_SYNC_DEFAULT_SAMPLE_RATE  1e6     /* 1 MSPS */
#define XMASS_SYNC_DEFAULT_BANDWIDTH    1e6     /* 1 MHz */
#define XMASS_SYNC_DEFAULT_GAIN         40.0    /* 40 dB */
#define XMASS_SYNC_DEFAULT_CAL_INTERVAL 1800    /* 30 minutes */

/*============================================================================
 * Error Codes
 *===========================================================================*/

typedef enum {
    XMASS_SYNC_OK = 0,
    XMASS_SYNC_ERR_INVALID_PARAM = -1,
    XMASS_SYNC_ERR_NOT_INITIALIZED = -2,
    XMASS_SYNC_ERR_DEVICE_ERROR = -3,
    XMASS_SYNC_ERR_CALIBRATION_FAILED = -4,
    XMASS_SYNC_ERR_NO_SIGNAL = -5,
    XMASS_SYNC_ERR_TIMEOUT = -6,
    XMASS_SYNC_ERR_MEMORY = -7,
} xmass_sync_error_t;

/*============================================================================
 * Calibration State
 *===========================================================================*/

typedef struct {
    double phase_offset[XMASS_SYNC_MAX_CHANNELS];       /* Phase offsets (radians) */
    double amplitude_correction[XMASS_SYNC_MAX_CHANNELS]; /* Amplitude corrections */
    double phase_stability[XMASS_SYNC_MAX_CHANNELS];    /* Phase stability (std dev) */
    double calibration_freq;                             /* Calibration frequency (Hz) */
    uint64_t calibration_timestamp;                      /* Calibration time (Unix epoch) */
    bool valid;                                          /* Calibration is valid */
    uint32_t calibration_count;                          /* Number of calibrations performed */
} xmass_sync_calibration_t;

/*============================================================================
 * Synchronization Configuration
 *===========================================================================*/

typedef struct {
    double sample_rate;             /* Sample rate (Hz) */
    double bandwidth;               /* Bandwidth (Hz) */
    double gain;                    /* Gain (dB) */
    uint32_t num_samples;           /* Samples per capture */
    double calibration_freq;        /* Calibration frequency (Hz) */
    uint32_t calibration_interval;  /* Re-calibration interval (seconds) */
    bool auto_recalibrate;          /* Enable automatic re-calibration */
} xmass_sync_config_t;

/*============================================================================
 * Synchronization State
 *===========================================================================*/

typedef struct {
    /* Carrier board interface */
    xmass_carrier_state_t *carrier;
    
    /* Configuration */
    xmass_sync_config_t config;
    
    /* Calibration data */
    xmass_sync_calibration_t calibration;
    
    /* Device handles (SoapySDR or usdr-lib) */
    void *devices[XMASS_SYNC_MAX_CHANNELS];
    void *streams[XMASS_SYNC_MAX_CHANNELS];
    
    /* State */
    bool initialized;
    bool streaming;
    int num_channels;
    
    /* Statistics */
    uint64_t samples_captured;
    uint64_t sync_events;
    
} xmass_sync_state_t;

/*============================================================================
 * Sample Buffer
 *===========================================================================*/

typedef struct {
    float complex *samples;         /* Complex sample data */
    size_t num_samples;             /* Number of samples */
    uint64_t timestamp;             /* Timestamp (if available) */
    uint8_t channel;                /* Channel index */
    bool valid;                     /* Buffer contains valid data */
} xmass_sync_buffer_t;

/*============================================================================
 * Initialization and Configuration
 *===========================================================================*/

/**
 * @brief Initialize the synchronization module
 * @param state Pointer to sync state structure
 * @param carrier Pointer to initialized carrier state
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_init(xmass_sync_state_t *state,
                                    xmass_carrier_state_t *carrier);

/**
 * @brief Deinitialize the synchronization module
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_deinit(xmass_sync_state_t *state);

/**
 * @brief Set synchronization configuration
 * @param state Pointer to sync state structure
 * @param config Configuration structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_configure(xmass_sync_state_t *state,
                                         const xmass_sync_config_t *config);

/**
 * @brief Get default configuration
 * @param config Pointer to store default configuration
 */
void xmass_sync_get_default_config(xmass_sync_config_t *config);

/*============================================================================
 * Device Management
 *===========================================================================*/

/**
 * @brief Open all xSDR devices
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_open_devices(xmass_sync_state_t *state);

/**
 * @brief Close all xSDR devices
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_close_devices(xmass_sync_state_t *state);

/**
 * @brief Configure all devices with identical settings
 * @param state Pointer to sync state structure
 * @param frequency Center frequency (Hz)
 * @param sample_rate Sample rate (Hz)
 * @param gain Gain (dB)
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_configure_devices(xmass_sync_state_t *state,
                                                 double frequency,
                                                 double sample_rate,
                                                 double gain);

/**
 * @brief Verify reference clock on all devices
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_verify_ref_clock(xmass_sync_state_t *state);

/*============================================================================
 * Streaming Control
 *===========================================================================*/

/**
 * @brief Setup RX streams on all devices
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_setup_streams(xmass_sync_state_t *state);

/**
 * @brief Start streaming on all devices
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_start_streaming(xmass_sync_state_t *state);

/**
 * @brief Stop streaming on all devices
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_stop_streaming(xmass_sync_state_t *state);

/**
 * @brief Capture samples from all devices
 * @param state Pointer to sync state structure
 * @param buffers Array of buffer pointers (one per channel)
 * @param num_samples Number of samples to capture
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_capture_samples(xmass_sync_state_t *state,
                                               xmass_sync_buffer_t *buffers,
                                               size_t num_samples);

/*============================================================================
 * Phase Calibration
 *===========================================================================*/

/**
 * @brief Perform phase calibration
 * @param state Pointer to sync state structure
 * @param calibration_freq Calibration signal frequency (Hz)
 * @return XMASS_SYNC_OK on success, error code on failure
 * 
 * @note Requires a strong CW signal at calibration_freq
 */
xmass_sync_error_t xmass_sync_calibrate_phase(xmass_sync_state_t *state,
                                               double calibration_freq);

/**
 * @brief Apply phase and amplitude corrections to buffers
 * @param state Pointer to sync state structure
 * @param buffers Array of buffer pointers to correct
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_apply_corrections(xmass_sync_state_t *state,
                                                 xmass_sync_buffer_t *buffers);

/**
 * @brief Get current calibration data
 * @param state Pointer to sync state structure
 * @param calibration Pointer to store calibration data
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_get_calibration(xmass_sync_state_t *state,
                                               xmass_sync_calibration_t *calibration);

/**
 * @brief Set calibration data (load from file or previous session)
 * @param state Pointer to sync state structure
 * @param calibration Calibration data to set
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_set_calibration(xmass_sync_state_t *state,
                                               const xmass_sync_calibration_t *calibration);

/**
 * @brief Check if calibration is still valid
 * @param state Pointer to sync state structure
 * @return true if calibration is valid, false otherwise
 */
bool xmass_sync_calibration_valid(xmass_sync_state_t *state);

/**
 * @brief Save calibration to file
 * @param state Pointer to sync state structure
 * @param filename Output file path
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_save_calibration(xmass_sync_state_t *state,
                                                const char *filename);

/**
 * @brief Load calibration from file
 * @param state Pointer to sync state structure
 * @param filename Input file path
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_load_calibration(xmass_sync_state_t *state,
                                                const char *filename);

/*============================================================================
 * Hardware Synchronization (requires LMK05318B access)
 *===========================================================================*/

/**
 * @brief Trigger hardware SYNC (if LMK05318B accessible)
 * @param state Pointer to sync state structure
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_hardware_sync(xmass_sync_state_t *state);

/**
 * @brief Configure LMK05318B for optimal synchronization
 * @param state Pointer to sync state structure
 * @param output_freq Desired output frequency (Hz)
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_configure_lmk(xmass_sync_state_t *state,
                                             double output_freq);

/*============================================================================
 * Coherence Verification
 *===========================================================================*/

/**
 * @brief Measure phase coherence between channels
 * @param state Pointer to sync state structure
 * @param phase_diffs Output array for phase differences (radians)
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_measure_coherence(xmass_sync_state_t *state,
                                                 double *phase_diffs);

/**
 * @brief Verify synchronization quality
 * @param state Pointer to sync state structure
 * @param max_phase_error Maximum acceptable phase error (radians)
 * @return true if sync quality is acceptable, false otherwise
 */
bool xmass_sync_verify_quality(xmass_sync_state_t *state, double max_phase_error);

/*============================================================================
 * Buffer Management
 *===========================================================================*/

/**
 * @brief Allocate sample buffer
 * @param buffer Pointer to buffer structure
 * @param num_samples Number of samples to allocate
 * @return XMASS_SYNC_OK on success, error code on failure
 */
xmass_sync_error_t xmass_sync_buffer_alloc(xmass_sync_buffer_t *buffer,
                                            size_t num_samples);

/**
 * @brief Free sample buffer
 * @param buffer Pointer to buffer structure
 */
void xmass_sync_buffer_free(xmass_sync_buffer_t *buffer);

/**
 * @brief Clear sample buffer
 * @param buffer Pointer to buffer structure
 */
void xmass_sync_buffer_clear(xmass_sync_buffer_t *buffer);

/*============================================================================
 * Diagnostics
 *===========================================================================*/

/**
 * @brief Print synchronization status
 * @param state Pointer to sync state structure
 */
void xmass_sync_print_status(xmass_sync_state_t *state);

/**
 * @brief Get error string
 * @param error Error code
 * @return Error description string
 */
const char* xmass_sync_error_string(xmass_sync_error_t error);

#ifdef __cplusplus
}
#endif

#endif /* XMASS_SYNC_H */
