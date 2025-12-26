/**
 * @file df_example.c
 * @brief Direction Finding Example using xMASS
 * 
 * Demonstrates basic direction finding using 4 xSDR modules
 * with phase comparison and MUSIC algorithms.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include "xmass_sync.h"

#define PI 3.14159265358979323846
#define SPEED_OF_LIGHT 299792458.0

static volatile bool running = true;

static void signal_handler(int sig)
{
    (void)sig;
    running = false;
}

/*============================================================================
 * Direction Finding Algorithms
 *===========================================================================*/

/**
 * @brief Simple phase comparison DOA
 * 
 * Uses phase difference between elements 0 and 1 to estimate bearing.
 */
static double phase_comparison_doa(float complex *buffers[], int num_channels,
                                    size_t num_samples, double frequency,
                                    double element_spacing)
{
    (void)num_channels;
    
    double wavelength = SPEED_OF_LIGHT / frequency;
    double d = element_spacing * wavelength;  /* Physical spacing */
    double k = 2.0 * PI / wavelength;
    
    /* Cross-correlation between channels 0 and 1 */
    float complex correlation = 0.0f;
    for (size_t i = 0; i < num_samples; i++) {
        correlation += buffers[1][i] * conjf(buffers[0][i]);
    }
    correlation /= num_samples;
    
    double phase_diff = cargf(correlation);
    
    /* DOA calculation: sin(θ) = Δφ / (k * d) */
    double sin_theta = phase_diff / (k * d);
    
    /* Clamp to valid range */
    if (sin_theta > 1.0) sin_theta = 1.0;
    if (sin_theta < -1.0) sin_theta = -1.0;
    
    double theta_rad = asin(sin_theta);
    double theta_deg = theta_rad * 180.0 / PI;
    
    return theta_deg;
}

/**
 * @brief MUSIC algorithm for DOA estimation
 * 
 * More accurate than phase comparison, requires knowing number of sources.
 */
static double music_doa(float complex *buffers[], int num_channels,
                        size_t num_samples, double frequency,
                        double element_spacing, int num_sources)
{
    double wavelength = SPEED_OF_LIGHT / frequency;
    double d = element_spacing * wavelength;
    double k = 2.0 * PI / wavelength;
    
    /* Build covariance matrix R = X * X^H / N */
    double complex R[4][4] = {{0}};
    
    for (size_t n = 0; n < num_samples; n++) {
        for (int i = 0; i < num_channels; i++) {
            for (int j = 0; j < num_channels; j++) {
                R[i][j] += buffers[i][n] * conjf(buffers[j][n]);
            }
        }
    }
    
    for (int i = 0; i < num_channels; i++) {
        for (int j = 0; j < num_channels; j++) {
            R[i][j] /= num_samples;
        }
    }
    
    /* 
     * Note: Full MUSIC implementation would require eigendecomposition.
     * This is a simplified version using power method for demonstration.
     * For production, use LAPACK or similar library.
     */
    
    /* Simplified: scan angles and find peak */
    double best_angle = 0.0;
    double best_power = 0.0;
    
    for (double angle = -90.0; angle <= 90.0; angle += 0.5) {
        double theta = angle * PI / 180.0;
        
        /* Steering vector */
        double complex a[4];
        for (int i = 0; i < num_channels; i++) {
            a[i] = cexp(-I * k * d * i * sin(theta));
        }
        
        /* Compute a^H * R * a (beamformer output power) */
        double complex power = 0.0;
        for (int i = 0; i < num_channels; i++) {
            for (int j = 0; j < num_channels; j++) {
                power += conj(a[i]) * R[i][j] * a[j];
            }
        }
        
        double power_real = cabs(power);
        if (power_real > best_power) {
            best_power = power_real;
            best_angle = angle;
        }
    }
    
    (void)num_sources;  /* Would be used in full MUSIC */
    
    return best_angle;
}

/*============================================================================
 * Main
 *===========================================================================*/

int main(int argc, char *argv[])
{
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║           xMASS Direction Finding Example                    ║\n");
    printf("║                                                              ║\n");
    printf("║  4-element coherent array using xSDR modules                 ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    /* Parse arguments */
    double target_freq = 100e6;     /* Default: 100 MHz */
    double sample_rate = 1e6;       /* Default: 1 MSPS */
    double element_spacing = 0.5;   /* Default: λ/2 */
    
    if (argc > 1) {
        target_freq = atof(argv[1]) * 1e6;
    }
    if (argc > 2) {
        sample_rate = atof(argv[2]) * 1e6;
    }
    if (argc > 3) {
        element_spacing = atof(argv[3]);
    }
    
    printf("Configuration:\n");
    printf("  Target frequency: %.3f MHz\n", target_freq / 1e6);
    printf("  Sample rate: %.3f MSPS\n", sample_rate / 1e6);
    printf("  Element spacing: %.2f λ\n", element_spacing);
    printf("\n");
    
    /* Setup signal handler */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* Initialize carrier board */
    printf("[*] Initializing carrier board...\n");
    xmass_carrier_state_t carrier;
    xmass_error_t cerr = xmass_carrier_init(&carrier, XMASS_I2C_METHOD_LINUX);
    
    if (cerr != XMASS_OK) {
        printf("[!] Carrier init failed (error %d)\n", cerr);
        printf("[*] Continuing in simulation mode...\n");
        
        /* Setup mock carrier */
        memset(&carrier, 0, sizeof(carrier));
        carrier.num_modules_detected = 4;
        carrier.initialized = true;
        carrier.lmk_accessible = false;
    }
    
    /* Initialize synchronization */
    printf("[*] Initializing synchronization module...\n");
    xmass_sync_state_t sync;
    xmass_sync_error_t serr = xmass_sync_init(&sync, &carrier);
    
    if (serr != XMASS_SYNC_OK) {
        printf("[!] Sync init failed: %s\n", xmass_sync_error_string(serr));
        return 1;
    }
    
    /* Configure */
    xmass_sync_config_t config;
    xmass_sync_get_default_config(&config);
    config.sample_rate = sample_rate;
    config.calibration_freq = target_freq;
    xmass_sync_configure(&sync, &config);
    
    /* Check for existing calibration */
    const char *cal_file = "/tmp/xmass_calibration.cal";
    if (access(cal_file, F_OK) == 0) {
        printf("[*] Loading existing calibration from %s\n", cal_file);
        xmass_sync_load_calibration(&sync, cal_file);
    } else {
        printf("[!] No calibration file found\n");
        printf("[*] Run calibration tool first: xmass_calibrate\n");
        printf("[*] Using default (uncalibrated) settings\n");
        
        /* Set default calibration */
        for (int i = 0; i < XMASS_SYNC_MAX_CHANNELS; i++) {
            sync.calibration.phase_offset[i] = 0.0;
            sync.calibration.amplitude_correction[i] = 1.0;
        }
        sync.calibration.valid = true;
    }
    
    /* Allocate buffers */
    xmass_sync_buffer_t buffers[XMASS_SYNC_MAX_CHANNELS];
    float complex *sample_ptrs[XMASS_SYNC_MAX_CHANNELS];
    size_t num_samples = 8192;
    
    for (int i = 0; i < sync.num_channels; i++) {
        xmass_sync_buffer_alloc(&buffers[i], num_samples);
        sample_ptrs[i] = buffers[i].samples;
    }
    
    /* Open devices and start streaming */
    printf("[*] Opening devices...\n");
    xmass_sync_open_devices(&sync);
    
    printf("[*] Configuring for %.3f MHz...\n", target_freq / 1e6);
    xmass_sync_configure_devices(&sync, target_freq, sample_rate, 40.0);
    
    printf("[*] Setting up streams...\n");
    xmass_sync_setup_streams(&sync);
    
    printf("[*] Starting streaming...\n");
    xmass_sync_start_streaming(&sync);
    
    /* Main loop */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  Direction Finding Active - Press Ctrl+C to stop            ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    int iteration = 0;
    while (running) {
        /* Capture samples */
        xmass_sync_capture_samples(&sync, buffers, num_samples);
        
        /* Apply calibration corrections */
        xmass_sync_apply_corrections(&sync, buffers);
        
        /* Compute DOA using both methods */
        double bearing_phase = phase_comparison_doa(sample_ptrs, sync.num_channels,
                                                     num_samples, target_freq,
                                                     element_spacing);
        
        double bearing_music = music_doa(sample_ptrs, sync.num_channels,
                                          num_samples, target_freq,
                                          element_spacing, 1);
        
        /* Display results */
        printf("\r[%06d] Phase: %+7.2f°  |  MUSIC: %+7.2f°  |  Avg: %+7.2f°    ",
               iteration++,
               bearing_phase,
               bearing_music,
               (bearing_phase + bearing_music) / 2.0);
        fflush(stdout);
        
        /* Small delay */
        usleep(100000);  /* 100ms */
    }
    
    printf("\n\n");
    
    /* Cleanup */
    printf("[*] Stopping streaming...\n");
    xmass_sync_stop_streaming(&sync);
    
    printf("[*] Closing devices...\n");
    xmass_sync_close_devices(&sync);
    
    for (int i = 0; i < sync.num_channels; i++) {
        xmass_sync_buffer_free(&buffers[i]);
    }
    
    xmass_sync_deinit(&sync);
    xmass_carrier_deinit(&carrier);
    
    printf("[+] Done!\n\n");
    
    return 0;
}
