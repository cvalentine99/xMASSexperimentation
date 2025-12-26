/**
 * @file calibrate.c
 * @brief Phase Calibration Tool for xMASS
 * 
 * Performs phase calibration across all 4 xSDR modules using
 * a known reference signal.
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>
#include "xmass_sync.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("\n");
    printf("Options:\n");
    printf("  -f, --frequency <MHz>   Calibration frequency (default: 100 MHz)\n");
    printf("  -o, --output <file>     Output calibration file (default: /tmp/xmass_calibration.cal)\n");
    printf("  -n, --snapshots <N>     Number of calibration snapshots (default: 10)\n");
    printf("  -s, --samples <N>       Samples per snapshot (default: 8192)\n");
    printf("  -g, --gain <dB>         Receiver gain (default: 40 dB)\n");
    printf("  -h, --help              Show this help message\n");
    printf("\n");
    printf("Example:\n");
    printf("  %s -f 100.0 -o /tmp/cal.txt\n", prog);
    printf("\n");
    printf("Note: Ensure a strong CW signal is present at the calibration frequency!\n");
    printf("\n");
}

int main(int argc, char *argv[])
{
    /* Default parameters */
    double cal_freq = 100e6;
    const char *output_file = "/tmp/xmass_calibration.cal";
    int num_snapshots = 10;
    int num_samples = 8192;
    double gain = 40.0;
    
    /* Parse command line */
    static struct option long_options[] = {
        {"frequency", required_argument, 0, 'f'},
        {"output",    required_argument, 0, 'o'},
        {"snapshots", required_argument, 0, 'n'},
        {"samples",   required_argument, 0, 's'},
        {"gain",      required_argument, 0, 'g'},
        {"help",      no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "f:o:n:s:g:h", long_options, NULL)) != -1) {
        switch (opt) {
            case 'f':
                cal_freq = atof(optarg) * 1e6;
                break;
            case 'o':
                output_file = optarg;
                break;
            case 'n':
                num_snapshots = atoi(optarg);
                break;
            case 's':
                num_samples = atoi(optarg);
                break;
            case 'g':
                gain = atof(optarg);
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║           xMASS Phase Calibration Tool                       ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("Configuration:\n");
    printf("  Calibration frequency: %.3f MHz\n", cal_freq / 1e6);
    printf("  Output file: %s\n", output_file);
    printf("  Snapshots: %d\n", num_snapshots);
    printf("  Samples per snapshot: %d\n", num_samples);
    printf("  Gain: %.1f dB\n", gain);
    printf("\n");
    
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  IMPORTANT: Ensure a strong CW signal is present at          ║\n");
    printf("║  %.3f MHz before continuing!                             ║\n", cal_freq / 1e6);
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("Press Enter to continue or Ctrl+C to abort...");
    getchar();
    printf("\n");
    
    /* Initialize carrier board */
    printf("[*] Initializing carrier board...\n");
    xmass_carrier_state_t carrier;
    xmass_error_t cerr = xmass_carrier_init(&carrier, XMASS_I2C_METHOD_LINUX);
    
    if (cerr != XMASS_OK) {
        printf("[!] Carrier init failed (error %d)\n", cerr);
        printf("[*] Continuing in simulation mode...\n");
        
        memset(&carrier, 0, sizeof(carrier));
        carrier.num_modules_detected = 4;
        carrier.initialized = true;
    }
    
    /* Print carrier status */
    xmass_carrier_print_status(&carrier);
    
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
    config.sample_rate = 1e6;
    config.gain = gain;
    config.num_samples = num_samples;
    config.calibration_freq = cal_freq;
    xmass_sync_configure(&sync, &config);
    
    /* Open devices */
    printf("[*] Opening devices...\n");
    serr = xmass_sync_open_devices(&sync);
    if (serr != XMASS_SYNC_OK) {
        printf("[!] Failed to open devices: %s\n", xmass_sync_error_string(serr));
        /* Continue anyway for testing */
    }
    
    /* Verify reference clock */
    printf("[*] Verifying reference clock...\n");
    xmass_sync_verify_ref_clock(&sync);
    
    /* Perform calibration */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  Starting Phase Calibration                                  ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    serr = xmass_sync_calibrate_phase(&sync, cal_freq);
    
    if (serr != XMASS_SYNC_OK) {
        printf("[!] Calibration failed: %s\n", xmass_sync_error_string(serr));
        /* Continue to save whatever we got */
    }
    
    /* Print detailed results */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  Calibration Results                                         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("Phase Offsets (relative to channel 0):\n");
    for (int i = 0; i < sync.num_channels; i++) {
        printf("  Channel %d: %+8.3f° (%+.6f rad)\n",
               i,
               sync.calibration.phase_offset[i] * 180.0 / M_PI,
               sync.calibration.phase_offset[i]);
    }
    
    printf("\nAmplitude Corrections:\n");
    for (int i = 0; i < sync.num_channels; i++) {
        printf("  Channel %d: %.4fx (%.2f dB)\n",
               i,
               sync.calibration.amplitude_correction[i],
               20.0 * log10(sync.calibration.amplitude_correction[i]));
    }
    
    printf("\nPhase Stability (standard deviation):\n");
    for (int i = 0; i < sync.num_channels; i++) {
        printf("  Channel %d: ±%.3f° (±%.6f rad)\n",
               i,
               sync.calibration.phase_stability[i] * 180.0 / M_PI,
               sync.calibration.phase_stability[i]);
    }
    
    /* Quality assessment */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  Quality Assessment                                          ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    double max_stability = 0.0;
    for (int i = 0; i < sync.num_channels; i++) {
        if (sync.calibration.phase_stability[i] > max_stability) {
            max_stability = sync.calibration.phase_stability[i];
        }
    }
    
    double stability_deg = max_stability * 180.0 / M_PI;
    
    if (stability_deg < 1.0) {
        printf("  [EXCELLENT] Phase stability < 1°\n");
        printf("  Calibration quality is excellent for direction finding.\n");
    } else if (stability_deg < 5.0) {
        printf("  [GOOD] Phase stability < 5°\n");
        printf("  Calibration quality is good for most applications.\n");
    } else if (stability_deg < 10.0) {
        printf("  [FAIR] Phase stability < 10°\n");
        printf("  Calibration may be acceptable, but accuracy will be limited.\n");
    } else {
        printf("  [POOR] Phase stability >= 10°\n");
        printf("  Calibration quality is poor. Check:\n");
        printf("  - Signal strength (should be > -40 dBm)\n");
        printf("  - Reference clock distribution\n");
        printf("  - Antenna connections\n");
    }
    
    /* Save calibration */
    printf("\n[*] Saving calibration to %s...\n", output_file);
    serr = xmass_sync_save_calibration(&sync, output_file);
    
    if (serr == XMASS_SYNC_OK) {
        printf("[+] Calibration saved successfully!\n");
    } else {
        printf("[!] Failed to save calibration: %s\n", xmass_sync_error_string(serr));
    }
    
    /* Print final status */
    printf("\n");
    xmass_sync_print_status(&sync);
    
    /* Cleanup */
    printf("[*] Cleaning up...\n");
    xmass_sync_close_devices(&sync);
    xmass_sync_deinit(&sync);
    xmass_carrier_deinit(&carrier);
    
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║  Calibration Complete!                                       ║\n");
    printf("║                                                              ║\n");
    printf("║  Use the calibration file with xmass_df_example:             ║\n");
    printf("║  $ xmass_df_example 100.0                                    ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    return 0;
}
