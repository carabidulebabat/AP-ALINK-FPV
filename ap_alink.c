#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sched.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/stat.h>

// Forward declarations for functions used early in the file
void setup_driver_power_management(void);

// Forward declarations for WiFi card specific functions
void pit_mode_enable_8812au(void);
void pit_mode_disable_8812au(void);
void pit_mode_enable_8812eu2(void);
void pit_mode_disable_8812eu2(void);
void pit_mode_enable_873xbu(void);
void pit_mode_disable_873xbu(void);
int parse_rssi_8812au_format(const char *buffer);
int parse_dbm_8812au_format(const char *buffer);
int parse_rssi_8812eu_format(const char *buffer);
int parse_dbm_8812eu_format(const char *buffer);
int parse_rssi_873xbu_format(const char *buffer);
int parse_dbm_873xbu_format(const char *buffer);
int execute_command(const char *command);

// Function pointer for RSSI format parser (set once during init)
static int (*rssi_format_parser)(const char *) = NULL;

// Function pointer for dBm format parser (set once during init)
static int (*dbm_format_parser)(const char *) = NULL;

// WiFi card configuration lookup table
typedef struct {
    const char* name;
    const char* driver_path;
    int hardware_mcs_offset;
    void (*pit_mode_enable_func)(void);
    void (*pit_mode_disable_func)(void);
    int (*rssi_format_parser)(const char* buffer);
    int (*dbm_format_parser)(const char* buffer);
    const char* power_save_command;
    const char* max_tx_power_command;
    const char* pit_tx_power_command;
    const char* regulatory_domain_command;
    const char* bitrate_command;
    const char* driver_reload_command;
    const char* power_save_description;
    const char* mcs_hardware_description;
    const char* modprobe_config_message;
} wifi_card_config_t;

// WiFi card configurations lookup table
static const wifi_card_config_t wifi_card_configs[] = {
    {
        .name = "8812eu2",
        .driver_path = "/proc/net/rtl88x2eu/wlan0",
        .hardware_mcs_offset = 2,
        .pit_mode_enable_func = pit_mode_enable_8812eu2,
        .pit_mode_disable_func = pit_mode_disable_8812eu2,
        .rssi_format_parser = parse_rssi_8812eu_format,
        .dbm_format_parser = parse_dbm_8812eu_format,
        .power_save_command = "iw dev wlan0 set power_save off",
        .max_tx_power_command = "iw dev wlan0 set txpower fixed 30",
        .pit_tx_power_command = "iw dev wlan0 set txpower fixed 10",
        .regulatory_domain_command = "iw reg set 00",
        .bitrate_command = "iw dev wlan0 set bitrates legacy-5 12 18 24 36 48 54",
        .driver_reload_command = "modprobe -r 8812eu2 && modprobe 8812eu2 rtw_power_mgnt=0 rtw_en_autosleep=0",
        .power_save_description = "RTL8812EU (8812eu2 driver)",
        .mcs_hardware_description = "RTL8812EU - Conservative thresholds (+2 dBm, maximum stability)",
        .modprobe_config_message = "options 8812eu2 rtw_power_mgnt=0 rtw_en_autosleep=0"
    },
    {
        .name = "8812au",
        .driver_path = "/proc/net/rtl88xxau/wlan0",
        .hardware_mcs_offset = 0,
        .pit_mode_enable_func = pit_mode_enable_8812au,
        .pit_mode_disable_func = pit_mode_disable_8812au,
        .rssi_format_parser = parse_rssi_8812au_format,
        .dbm_format_parser = parse_dbm_8812au_format,
        .power_save_command = "iw dev wlan0 set power_save off",
        .max_tx_power_command = "iw dev wlan0 set txpower fixed 30",
        .pit_tx_power_command = "iw dev wlan0 set txpower fixed 10",
        .regulatory_domain_command = "iw reg set 00",
        .bitrate_command = "iw dev wlan0 set bitrates legacy-5 12 18 24 36 48 54",
        .driver_reload_command = "modprobe -r 88XXau && modprobe 88XXau rtw_power_mgnt=0",
        .power_save_description = "RTL8812AU (8812au driver)",
        .mcs_hardware_description = "RTL8812AU - Aggressive thresholds (maximum performance)",
        .modprobe_config_message = "options 88XXau rtw_power_mgnt=0"
    },
    {
        .name = "873xbu",
        .driver_path = "/proc/net/rtl873xbu/wlan0",
        .hardware_mcs_offset = 1,
        .pit_mode_enable_func = pit_mode_enable_873xbu,
        .pit_mode_disable_func = pit_mode_disable_873xbu,
        .rssi_format_parser = parse_rssi_873xbu_format,
        .dbm_format_parser = parse_dbm_873xbu_format,
        .power_save_command = "iw dev wlan0 set power_save off",
        .max_tx_power_command = "iw dev wlan0 set txpower fixed 20",
        .pit_tx_power_command = "iw dev wlan0 set txpower fixed 5",
        .regulatory_domain_command = "iw reg set 00",
        .bitrate_command = "iw dev wlan0 set bitrates legacy-5 12 18 24 36 48 54",
        .driver_reload_command = "modprobe -r 873xbu && modprobe 873xbu rtw_power_mgnt=0 rtw_en_autosleep=0",
        .power_save_description = "RTL873xBU (873xbu driver)",
        .mcs_hardware_description = "RTL873xBU - Balanced thresholds (+1 dBm, optimal balance)",
        .modprobe_config_message = "options 873xbu rtw_power_mgnt=0 rtw_en_autosleep=0"
    }
};

// Pre-calculated constants for lookup table sizes (never change at runtime)
static const int WIFI_CARD_CONFIGS_TABLE_SIZE = sizeof(wifi_card_configs) / sizeof(wifi_card_configs[0]);

// Helper function to find WiFi card configuration
static const wifi_card_config_t* get_wifi_card_config(const char* card_name) {
    for (int i = 0; i < WIFI_CARD_CONFIGS_TABLE_SIZE; i++) {
        if (strcmp(wifi_card_configs[i].name, card_name) == 0) {
            return &wifi_card_configs[i];
        }
    }
    return NULL;
}

// =============================================================================
// GLOBAL CONFIGURATION VALUES (Runtime Configurable)
// =============================================================================

// System Configuration Constants (Runtime Configurable)
#define MAX_COMMAND_LENGTH 1024
#define COMMAND_BUFFER_SIZE 128
static int HTTP_TIMEOUT_US = 100000;  // 100ms timeout
static int HTTP_PORT = 80;
static int REALTIME_PRIORITY = 10;
static int SAFETY_MARGIN_DBM = 3;  // Safety margin for emergency drop

// Bitrate Conversion Constants (Runtime Configurable)
static int BITRATE_MBPS_TO_KBPS = 1024;

// Signal Processing Defaults (Runtime Configurable)
static float DEFAULT_RSSI_ESTIMATE = 50.0f;
static float DEFAULT_DBM_ESTIMATE = -60.0f;
static float DEFAULT_ERROR_ESTIMATE = 1.0f;
// Default Configuration Values (used as initializers)
#define DEFAULT_PROCESS_VARIANCE 1e-5f
#define DEFAULT_RSSI_MEASUREMENT_VARIANCE 0.1f
#define DEFAULT_DBM_MEASUREMENT_VARIANCE 0.5f
#define DEFAULT_FILTER_ALPHA 0.1f
#define DEFAULT_CUTOFF_FREQ 2.0f
#define DEFAULT_SAMPLE_FREQ 10.0f
static int DEFAULT_MODE_FILTER_RETURN_ELEMENT = 2;  // Median (middle element)

// Filter Algorithm Constants (Runtime Configurable)
static float KALMAN_MIN_ERROR_ESTIMATE = 1e-6f;  // Minimum error estimate for numerical stability
static float LPF_2POLE_Q_FACTOR = 0.707f;  // Q factor for Butterworth response
static float GAUSSIAN_SIGMA = 1.0f;  // Standard deviation for Gaussian filter weights

// Main Loop Defaults (Runtime Configurable)
static int DEFAULT_BITRATE = 4;
static int DEFAULT_BITRATE_MIN = 1;
static int DEFAULT_DBM_MAX = -50;
// Removed DEFAULT_DBM_MIN - not used in calculations
static int DEFAULT_HYSTERESIS = 1;
static int DEFAULT_MINUS_HYSTERESIS = -1;
static int DEFAULT_INITIAL_DBM = -100;

// Signal Thresholds (Runtime Configurable - High Frequency Access)
static int HIGH_RSSI_THRESHOLD = 55;
static int MEDIUM_RSSI_THRESHOLD = 40;
static int LOW_RSSI_THRESHOLD = 20;
static int VLQ_MAX_THRESHOLD = 100;
static int VLQ_MIN_THRESHOLD = 1;
static int PERCENTAGE_CONVERSION = 100;

// Precomputed constants for hot path optimization
static double bitrate_scale = 0.0;           // Precomputed: bitrate_max / PERCENTAGE_CONVERSION
static long signal_sampling_interval_ns = 0; // Precomputed: NS_PER_SEC / signal_sampling_freq_hz
static const long NS_PER_SEC = 1000000000L;   // Nanoseconds per second constant

// Counter-based sampling optimization variables
static unsigned long sample_counter = 0;      // Simple counter for signal sampling
static unsigned long samples_per_loop = 0;    // Precomputed: 1000 / signal_sampling_freq_hz
static bool new_signal_data_available = false; // Flag for new signal data

// Precomputed derived values for configuration display
static float signal_sampling_freq_hz_display = 0.0f;  // Precomputed: fps / signal_sampling_interval
static float emergency_cooldown_frames_display = 0.0f; // Precomputed: emergency_cooldown_ms * fps / 1000.0

// Precomputed VLQ calculation constants for all possible dbm_Min values
static double dbm_range_inv_high = 0.0;      // For DBM_THRESHOLD_HIGH
static double dbm_range_inv_medium = 0.0;    // For DBM_THRESHOLD_MEDIUM  
static double dbm_range_inv_low = 0.0;       // For DBM_THRESHOLD_LOW
static double dbm_range_inv_fallback = 0.0;  // For DEFAULT_DBM_MAX - MIN_DBM_DIFFERENCE

// dBm Threshold Values (Runtime Configurable - High Frequency Access)
static int DBM_THRESHOLD_HIGH = -70;
static int DBM_THRESHOLD_MEDIUM = -55;
static int DBM_THRESHOLD_LOW = -53;
static int MIN_DBM_DIFFERENCE = 1;

// Bitrate to MCS mapping thresholds (Mbps) - Actual MCS data rates for 20MHz
static const double BITRATE_MCS_THRESHOLDS[] = {
    6.5, 13, 19.5, 26, 39, 52, 58.5, 65, 78, 86.7  // Mbps thresholds for MCS 0-9
};

// Configuration Variables (loaded from .conf file)
static int bitrate_max = 19;
static char wificard[16] = "8812eu2";
static int fps = 120;
static char rssi_filter_chain_config[64] = "0";
static char dbm_filter_chain_config[64] = "0";
static float lpf_cutoff_freq = DEFAULT_CUTOFF_FREQ;
static float lpf_sample_freq = DEFAULT_SAMPLE_FREQ;
static float kalman_rssi_process = DEFAULT_PROCESS_VARIANCE;
static float kalman_dbm_process = DEFAULT_PROCESS_VARIANCE;
static float kalman_rssi_measure = DEFAULT_RSSI_MEASUREMENT_VARIANCE;
static float kalman_dbm_measure = DEFAULT_DBM_MEASUREMENT_VARIANCE;
static int signal_sampling_interval = 5;
static int signal_sampling_freq_hz = 50;
static int cooldown_enabled = 0;
static unsigned long strict_cooldown_ms = 200;
static unsigned long up_cooldown_ms = 3000;
static unsigned long emergency_cooldown_ms = 50;
static int min_change_percent = 5;
static int emergency_rssi_threshold = 30;
static int emergency_bitrate = 1000;
static int hardware_rssi_offset = 0;
static int control_algorithm = 1;
static float pid_kp = 1.0f;
static float pid_ki = 0.05f;
static float pid_kd = 0.4f;
static int race_mode = 1;
static int disable_wifi_power_save = 1;  // Default: disable power saving for FPV stability
static int enable_maximum_tx_power = 1;  // Default: enable maximum TX power for FPV range
static int pit_mode_enabled = 0;         // PIT mode: low power standby with HTTP wake-up
static int hardware_mcs_offset = 0;      // Hardware-specific MCS threshold offset (set during init)
static char racing_video_resolution[32] = "1280x720";
static int racing_exposure = 11;
static int racing_fps = 120;
static int auto_exposure_enabled = 1;  // Enable automatic exposure calculation
static char racing_rssi_filter_chain_config[64] = "1";

// Performance optimization lookup tables
static int exposure_lookup_table[301];  // FPS 1-300 -> exposure values
static int mcs_command_lookup_table[16]; // MCS 0-15 -> command values
static int rssi_threshold_lookup_table[10]; // MCS 0-9 -> RSSI thresholds
static int bitrate_to_mcs_lookup_table[1001]; // Bitrate 0-1000 Mbps -> MCS values
static char racing_dbm_filter_chain_config[64] = "1";
static int loop_timing_mode = 0;  // 0=normal, 1=high_perf, 2=ultra_low_latency

// Driver path - determined once at startup based on WiFi card type
static char driverpath[256] = {0};

// =============================================================================
// WIFI CARD INITIALIZATION
// =============================================================================

void set_maximum_tx_power(void);
extern int wifi_driver_available;

/**
 * Initialize WiFi card with all necessary configurations
 * 
 * This function consolidates all WiFi card setup including:
 * - Driver path detection (always needed for MCS commands)
 * - Driver path validation (only for RSSI reading - non-critical)
 * - Hardware-specific MCS configuration (always needed)
 * - Maximum TX power setup (always needed)
 * - Power saving disable (always needed)
 * 
 * NOTE: Driver path validation failure does NOT prevent initialization.
 * MCS configuration and power management are always applied.
 * Only RSSI reading is affected by driver path issues.
 * 
 * @param wificard_type WiFi card type string (e.g., "8812eu2", "8812au", "873xbu")
 * @param driverpath Output buffer for driver path
 * @return 1 if initialization successful, 0 if failed (only on invalid card type)
 */
static int wifi_card_init(const char* wificard_type, char* driverpath) {
    printf("Initializing WiFi card: %s\n", wificard_type);
    
    // =============================================================================
    // 1. DRIVER PATH DETECTION USING LOOKUP TABLE
    // =============================================================================
    const wifi_card_config_t* config = get_wifi_card_config(wificard_type);
    if (!config) {
        printf("ERROR: Unknown WiFi card: %s\n", wificard_type);
        printf("Available options: ");
        for (int i = 0; i < WIFI_CARD_CONFIGS_TABLE_SIZE; i++) {
            printf("%s", wifi_card_configs[i].name);
            if (i < WIFI_CARD_CONFIGS_TABLE_SIZE - 1) {
                printf(", ");
            }
        }
        printf("\n");
        return 0;
    }
    
    // Set driver path from lookup table
    strcpy(driverpath, config->driver_path);
    printf("Driver path: %s\n", driverpath);
    
    // =============================================================================
    // 2. DRIVER PATH VALIDATION (Only if RSSI reading is enabled)
    // =============================================================================
    char test_path[512];
    snprintf(test_path, sizeof(test_path), "%s/sta_tp_info", driverpath);
    if (access(test_path, R_OK) != 0) {
        printf("WARNING: Driver path does not exist: %s\n", test_path);
        printf("RSSI reading will be disabled - bitrate control will use fallback methods\n");
        printf("Common solutions:\n");
        printf("  1. Change wificard=8812au in config\n");
        printf("  2. Check if WiFi is enabled: iwconfig\n");
        printf("  3. Verify driver is loaded: lsmod | grep 88\n");
        wifi_driver_available = 0;
    } else {
        printf("Driver path verified: %s\n", test_path);
        wifi_driver_available = 1;
    }
    
    // =============================================================================
    // 3. HARDWARE-SPECIFIC MCS CONFIGURATION USING LOOKUP TABLE
    // =============================================================================
    hardware_mcs_offset = config->hardware_mcs_offset;
    
    // Print hardware-specific MCS configuration using lookup table
    printf("Hardware MCS: %s\n", config->mcs_hardware_description);
    
    // Set format parsers from lookup table
    rssi_format_parser = config->rssi_format_parser;
    dbm_format_parser = config->dbm_format_parser;
    
    // =============================================================================
    // 4. POWER MANAGEMENT CONFIGURATION
    // =============================================================================
    printf("Configuring WiFi power management...\n");
    

    
    // Set maximum transmission power for maximum FPV range
    set_maximum_tx_power();
    
    // Setup driver-level power management (additional protection)
    setup_driver_power_management();
    
    // =============================================================================
    // 5. NETWORK BUFFER OPTIMIZATION (General WiFi Performance)
    // =============================================================================

    printf("Optimizing network buffers for WiFi performance...\n");
    int result1 = execute_command("sysctl -w net.core.rmem_default=16384");
    int result2 = execute_command("sysctl -w net.core.rmem_max=65536");
    int result3 = execute_command("sysctl -w net.core.wmem_default=16384");
    int result4 = execute_command("sysctl -w net.core.wmem_max=65536");
    int result5 = execute_command("ifconfig wlan0 txqueuelen 100");
    int result6 = execute_command("sysctl -w net.core.netdev_max_backlog=64");
    
    // Check if any network configuration commands failed
    if (result1 != 0 || result2 != 0 || result3 != 0 || result4 != 0 || result5 != 0 || result6 != 0) {
#ifdef DEBUG
        printf("Warning: Some network buffer configuration commands failed\n");
#endif
    } else {
        printf("Network buffers optimized for WiFi performance\n");
    }
    
    // =============================================================================
    // DEVICE-SPECIFIC OPTIMIZATIONS
    // =============================================================================
    printf("Applying device-specific optimizations...\n");
    
    // Get WiFi card configuration from lookup table
    const wifi_card_config_t* wifi_config = get_wifi_card_config(wificard);
    if (!wifi_config) {
        printf("ERROR: Unknown WiFi card type for optimizations: %s\n", wificard);
    } else {
        // Set regulatory domain (device-specific)
        int reg_result = execute_command(wifi_config->regulatory_domain_command);
        if (reg_result != 0) {
            printf("WARNING: Could not set regulatory domain\n");
        } else {
            printf("Regulatory domain set successfully\n");
        }
        
        // Set maximum TX power (device-specific)
        int max_power_result = execute_command(wifi_config->max_tx_power_command);
        if (max_power_result != 0) {
            printf("WARNING: Could not set maximum TX power\n");
        } else {
            printf("Maximum TX power set successfully\n");
        }
        
        // Optimize bitrates (device-specific)
        int bitrate_result = execute_command(wifi_config->bitrate_command);
        if (bitrate_result != 0) {
            printf("WARNING: Could not set optimized bitrates\n");
        } else {
            printf("Optimized bitrates set successfully\n");
        }
    }
    
    printf("Device-specific optimizations complete\n");
    
    printf("WiFi Access Point: Initializing APFPV access point...\n");
    
    // Start the WiFi adapter (sets up OpenIPC access point)
    int adapter_result = execute_command("adapter start");
    if (adapter_result != 0) {
        printf("WARNING: Failed to start WiFi adapter (exit code: %d)\n", adapter_result);
    } else {
        printf("WiFi Access Point: Adapter started successfully\n");
    }
    sleep(2);
    
    // Start DHCP server for client connections
    int dhcp_result = execute_command("udhcpd /etc/udhcpd.conf");
    if (dhcp_result != 0) {
        printf("WARNING: Failed to start DHCP server (exit code: %d)\n", dhcp_result);
    } else {
        printf("WiFi Access Point: DHCP server started successfully\n");
    }
    sleep(1);
    
    
    // Verify access point is running
    printf("WiFi Access Point Status:\n");
    execute_command("iw dev wlan0 info | grep -E 'ssid|channel|type|txpower'");
    
    printf("WiFi Access Point: Initialization complete\n");
 
    
    printf("WiFi card initialization complete\n");
    return 1;
}

// =============================================================================

/**
 * Calculate optimal exposure time based on frame rate using 180° shutter rule
 * 
 * NOTE: This is a cinematography guideline for global shutter cameras.
 * CMOS rolling shutter cameras (like IMX415) can set exposure independently of frame rate.
 * This function provides a starting point, but exposure should be optimized for:
 * - Racing: Short exposure for minimal motion blur
 * - Low Light: Longer exposure for brightness  
 * - Latency: Short exposure for responsiveness
 * 
 * @param fps Frame rate in frames per second
 * @return Calculated exposure time in milliseconds
 */
static int calculate_exposure_from_fps(int fps) {
    // Use lookup table for performance
    if (fps >= 0 && fps <= 300) {
        return exposure_lookup_table[fps];
    }
    
    // Fallback for out-of-range values
    if (fps <= 0) {
        return 8; // Default fallback
    }
    
    // Calculate frame period in milliseconds
    float frame_period_ms = 1000.0f / fps;
    
    // Apply 180° shutter rule: exposure = frame_period / 2
    float exposure_ms = frame_period_ms / 2.0f;
    
    // Round to nearest integer and ensure minimum of 1ms
    int exposure = (int)(exposure_ms + 0.5f);
    if (exposure < 1) exposure = 1;
    
    return exposure;
}

/**
 * Calculate optimal exposure time for racing mode
 * Uses 180° shutter rule with frame rate from racing_fps
 * 
 * @return Calculated exposure time in milliseconds
 */
static int calculate_racing_exposure(void) {
    return calculate_exposure_from_fps(racing_fps);
}

/**
 * Calculate optimal exposure time for normal mode
 * Uses 180° shutter rule with frame rate from fps
 * 
 * @return Calculated exposure time in milliseconds
 */
static int calculate_normal_exposure(void) {
    return calculate_exposure_from_fps(fps);
}

// =============================================================================
// RUNTIME CONFIGURATION FUNCTIONS
// =============================================================================

// =============================================================================

// Function declarations
unsigned long get_current_time_ms(void);

// Kalman Filter Structure
typedef struct {
    float estimate;           // Current estimate
    float error_estimate;     // Current error estimate
    float process_variance;   // Process noise variance
    float measurement_variance; // Measurement noise variance
} kalman_filter_t;

// Control Algorithm Types
typedef enum {
    CONTROL_ALGORITHM_PID = 0,      // PID controller (complex, smooth)
    CONTROL_ALGORITHM_FIFO = 1      // Simple FIFO (fast, direct)
} control_algorithm_t;

// PID Controller Structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Integral accumulator
    int last_error;     // Previous error for derivative calculation
    int last_output;    // Previous output for reference
} pid_controller_t;

// Sleep Configuration Structure
typedef struct {
    unsigned int main_loop_us;        // Base loop sleep (microseconds)
    unsigned int normal_mode_ms;      // Additional sleep for normal mode (milliseconds)
    unsigned int high_perf_mode_ms;   // Additional sleep for high performance mode (milliseconds)
    unsigned int ultra_low_mode_ms;   // Additional sleep for ultra-low latency mode (milliseconds)
    unsigned int error_condition_ms; // Sleep when RSSI=0 or driver unavailable (milliseconds)
    int smart_sleep_enabled;          // Enable/disable smart sleep (0=off, 1=on)
} sleep_config_t;

// Forward declaration of global sleep configuration
extern sleep_config_t sleep_config;

// Asymmetric Cooldown Constants - replaced by static variables

// Filter Types
typedef enum {
    FILTER_TYPE_KALMAN = 0,
    FILTER_TYPE_LOWPASS = 1,
    FILTER_TYPE_MODE = 2,
    FILTER_TYPE_DERIVATIVE = 3,
    FILTER_TYPE_2POLE_LPF = 4,
    FILTER_TYPE_MEAN = 5,
    FILTER_TYPE_GAUSSIAN = 6
} filter_type_t;

// Function pointer types for filter operations
typedef float (*filter_process_func_t)(float sample, void* filter_data, bool is_rssi);
typedef void (*filter_init_func_t)(void* filter_data, bool is_rssi);
typedef void (*filter_reset_func_t)(void* filter_data, bool is_rssi);

// Filter operation lookup table entry
typedef struct {
    filter_process_func_t process_func;
    filter_init_func_t init_func;
    filter_reset_func_t reset_func;
    const char* name;
} filter_lookup_entry_t;

// ArduPilot-style Low-Pass Filter Structure
typedef struct {
    float output;           // Current filtered output
    float alpha;           // Filter coefficient (0-1)
    bool initialised;      // Initialization flag
    float cutoff_freq;     // Cutoff frequency (Hz)
    float sample_freq;     // Sample frequency (Hz)
} lowpass_filter_t;

// Mode Filter Structure (ArduPilot style - median-like with alternating drop)
#define MODE_FILTER_SIZE 5
typedef struct {
    float samples[MODE_FILTER_SIZE];
    uint8_t sample_index;
    uint8_t return_element;
    bool drop_high_sample;
    float output;
} mode_filter_t;

// Mean Filter Structure (Simple moving average)
#define MEAN_FILTER_SIZE 8
typedef struct {
    float samples[MEAN_FILTER_SIZE];
    uint8_t sample_index;
    uint8_t sample_count;
    float sum;
    float output;
} mean_filter_t;

// Gaussian Filter Structure (Gaussian-weighted moving average)
#define GAUSSIAN_FILTER_SIZE 7
typedef struct {
    float samples[GAUSSIAN_FILTER_SIZE];
    float weights[GAUSSIAN_FILTER_SIZE];
    uint8_t sample_index;
    uint8_t sample_count;
    float output;
    bool weights_initialized;  // Flag to track if weights are pre-calculated
} gaussian_filter_t;

// Derivative Filter Structure (for trend detection)
#define DERIVATIVE_FILTER_SIZE 5
typedef struct {
    float samples[DERIVATIVE_FILTER_SIZE];
    uint32_t timestamps[DERIVATIVE_FILTER_SIZE];
    uint8_t sample_index;
    float last_slope;
    bool new_data;
} derivative_filter_t;

// 2-Pole Low-Pass Filter Structure (biquad filter)
typedef struct {
    float delay_element_1;
    float delay_element_2;
    float cutoff_freq;
    float sample_freq;
    bool initialised;
    float output;
    // Pre-calculated coefficients for performance
    float b0, b1, b2;  // Feedforward coefficients
    float a1, a2;      // Feedback coefficients (a0 normalized to 1.0)
} lpf_2pole_t;

// Filter Chain Structure - allows multiple filters in sequence
#define MAX_FILTERS_PER_CHAIN 3
typedef struct {
    filter_type_t filters[MAX_FILTERS_PER_CHAIN];
    uint8_t filter_count;
    bool enabled;
} filter_chain_t;

// Active filter chains (switched by toggle_racemode)
static filter_chain_t *active_rssi_filter_chain;
static filter_chain_t *active_dbm_filter_chain;

// Worker Thread Infrastructure Structures
typedef struct {
    int bitrateMcs;
    char mcspath[128];
} mcs_arg_t;

typedef enum {
    CMD_SET_BITRATE,
    CMD_SET_MCS
} cmd_type_t;

typedef struct {
    cmd_type_t type;
    union {
        int bitrate_kbps;
        mcs_arg_t mcs_data;
    } data;
} worker_cmd_t;

// Filter Chain Instances
static filter_chain_t rssi_filter_chain = {
    .filters = {FILTER_TYPE_KALMAN},
    .filter_count = 1,
    .enabled = true
};

static filter_chain_t dbm_filter_chain = {
    .filters = {FILTER_TYPE_KALMAN},
    .filter_count = 1,
    .enabled = true
};

// Global Racing Filter Chain instances (for race_mode=1)
static filter_chain_t rssi_race_filter_chain = {
    .filters = {FILTER_TYPE_LOWPASS},
    .filter_count = 1,
    .enabled = true
};

static filter_chain_t dbm_race_filter_chain = {
    .filters = {FILTER_TYPE_LOWPASS},
    .filter_count = 1,
    .enabled = true
};

// Function declarations for filter operations
float apply_filter_chain(filter_chain_t *chain, float sample);
void set_filter_chain(const char *config_str, filter_chain_t *chain);
void reset_filter_chain(filter_chain_t *chain);
void init_filter_chain_filters(filter_chain_t *chain, float process_var, float measure_var,
                               float lpf_cutoff_freq, float lpf_sample_freq, bool is_rssi);
void init_debug_output(void);
void init_cooldown_check(void);
void init_worker_thread(void);
void *worker_thread_func(void *arg);
void init_counter_sampling(void);
bool should_sample_signal_counter(void);
bool should_change_bitrate(int new_bitrate, int current_bitrate, unsigned long strict_cooldown_ms, 
                          unsigned long up_cooldown_ms, int min_change_percent, unsigned long emergency_cooldown_ms);
void toggle_racemode(void);
int toggle_racemode_http(void);
void set_maximum_tx_power(void);
void enable_pit_mode(void);
void disable_pit_mode(void);
void toggle_pit_mode(void);
int toggle_pit_mode_http(void);
int http_get(const char *path);

// Dynamic RSSI threshold functions
int get_dynamic_rssi_threshold(int current_mcs);
int bitrate_to_mcs(int bitrate_mbps);

// Sleep function prototypes
void simple_sleep(bool is_error_condition, bool has_work_done);
void smart_sleep(bool is_error_condition, bool has_work_done);
void main_loop_sleep(bool is_error_condition, bool has_work_done);
void init_sleep_values();
void enable_global_debug();

// RSSI reading function prototypes
int get_rssi_file_rewind(const char *readcmd);
int get_rssi_mmap(const char *readcmd);
int get_rssi(const char *readcmd);
void init_rssi_read_method();

// dBm reading function prototypes
int get_dbm_file_rewind(const char *readcmd);
int get_dbm_mmap(const char *readcmd);
int get_dbm(const char *readcmd);
void init_dbm_read_method();

// Control algorithm function prototypes
void init_control_algorithm();
int pid_control_algorithm(int target_bitrate, int last_bitrate, pid_controller_t *pid);
int fifo_control_algorithm(int target_bitrate, int last_bitrate, pid_controller_t *pid);
void pid_init(pid_controller_t *pid, float kp, float ki, float kd);
void pid_reset(pid_controller_t *pid);

// PIT mode function prototypes
void init_pit_mode();
void init_race_mode();
void init_http_system();
void init_wifi_access_point();
void pit_mode_enable_8812au(void);
void pit_mode_disable_8812au(void);
void pit_mode_enable_8812eu2(void);
void pit_mode_disable_8812eu2(void);
void pit_mode_enable_873xbu(void);
void pit_mode_disable_873xbu(void);


// Function prototypes for functions used by FEC
int execute_command(const char *command);

// Lookup table initialization functions
void build_exposure_lookup_table(void);
void build_mcs_command_lookup_table(void);
void build_rssi_threshold_lookup_table(void);
void build_bitrate_to_mcs_lookup_table(void);
void init_performance_lookup_tables(void);

// Pure function prototypes
int bitrate_to_mcs_pure(int bitrate_mbps, const double* thresholds);
int get_dynamic_rssi_threshold_pure(int current_mcs, 
                                   const int* mcs_thresholds,
                                   int hardware_rssi_offset,
                                   int hardware_mcs_offset);
double calculate_vlq_pure(int filtered_dbm, int dbm_min, int dbm_max, int filtered_rssi);
int calculate_target_bitrate_pure(double vlq, int bitrate_max);
int calculate_exposure_pure(int fps);
int get_mcs_command_pure(int mcs_index, const char* wificard_type);

// RSSI format parser prototypes
int parse_rssi_8812eu_format(const char *buffer);
int parse_rssi_8812au_format(const char *buffer);
int parse_rssi_873xbu_format(const char *buffer);

// dBm format parser prototypes
int parse_dbm_8812eu_format(const char *buffer);
int parse_dbm_8812au_format(const char *buffer);
int parse_dbm_873xbu_format(const char *buffer);

// Global high-performance debug logging system
#ifdef DEBUG
static char global_debug_buffer[2048];  // Larger buffer for entire application
static int global_debug_pos = 0;
static int global_debug_iteration = 0;
#endif
static bool global_debug_enabled = false;  // Set to true after initialization

// Global debug macros for entire application
#ifdef DEBUG
#define GLOBAL_DEBUG_RESET() global_debug_pos = 0
#define GLOBAL_DEBUG_APPEND(fmt, ...) \
    if (global_debug_enabled) { \
        global_debug_pos += snprintf(global_debug_buffer + global_debug_pos, \
                                   sizeof(global_debug_buffer) - global_debug_pos, \
                                   fmt, ##__VA_ARGS__); \
    }
#define GLOBAL_DEBUG_FLUSH() \
    if (global_debug_enabled && global_debug_pos > 0) { \
        printf("%s", global_debug_buffer); \
        global_debug_pos = 0; \
    }
#define GLOBAL_DEBUG_THROTTLE(n) (++global_debug_iteration % (n) == 0)
#define GLOBAL_DEBUG_BUILD(condition, fmt, ...) \
    if (global_debug_enabled && (condition)) { \
        GLOBAL_DEBUG_APPEND(fmt, ##__VA_ARGS__); \
    }
#else
// Dummy macros for non-debug builds
#define GLOBAL_DEBUG_RESET()
#define GLOBAL_DEBUG_APPEND(fmt, ...)
#define GLOBAL_DEBUG_FLUSH()
#define GLOBAL_DEBUG_THROTTLE(n) (0)
#define GLOBAL_DEBUG_BUILD(condition, fmt, ...)
#endif

// Enable global debug after initialization is complete
void enable_global_debug() {
    global_debug_enabled = true;
    printf("Global debug system enabled\n");
}

// =============================================================================
// PERFORMANCE OPTIMIZATION LOOKUP TABLES
// =============================================================================

// Build exposure lookup table for FPS 1-300
void build_exposure_lookup_table(void) {
    printf("Building exposure lookup table for FPS 1-300\n");
    
    for (int fps = 0; fps <= 300; fps++) {
        if (fps <= 0) {
            exposure_lookup_table[fps] = 8; // Default fallback
        } else {
            // Calculate frame period in milliseconds
            float frame_period_ms = 1000.0f / fps;
            
            // Apply 180° shutter rule: exposure = frame_period / 2
            float exposure_ms = frame_period_ms / 2.0f;
            
            // Round to nearest integer and ensure minimum of 1ms
            int exposure = (int)(exposure_ms + 0.5f);
            if (exposure < 1) exposure = 1;
            
            exposure_lookup_table[fps] = exposure;
        }
    }
    
    printf("Exposure lookup table built (FPS 0-300)\n");
}

// Build MCS command lookup table for MCS 0-15
void build_mcs_command_lookup_table(void) {
    printf("Building MCS command lookup table for MCS 0-15\n");
    
    for (int mcs = 0; mcs < 16; mcs++) {
        // Realtek MCS encoding: MCS 0-15 = 0x80 + mcs_index
        mcs_command_lookup_table[mcs] = 0x80 + mcs;
    }
    
    printf("MCS command lookup table built (MCS 0-15)\n");
}

// Build RSSI threshold lookup table for MCS 0-9
void build_rssi_threshold_lookup_table(void) {
    printf("Building RSSI threshold lookup table for MCS 0-9\n");
    
    // MCS-specific RSSI thresholds (lower MCS = more sensitive threshold)
    int mcs_thresholds[10] = {-95, -90, -85, -80, -75, -70, -65, -60, -55, -50};
    
    for (int mcs = 0; mcs < 10; mcs++) {
        int base_threshold = mcs_thresholds[mcs];
        
        // Apply hardware-specific offset
        int dynamic_threshold = base_threshold + hardware_rssi_offset;
        
        // Apply hardware-specific MCS offset (set during initialization)
        dynamic_threshold += hardware_mcs_offset;
        
        // Add safety margin for emergency drop
        dynamic_threshold += SAFETY_MARGIN_DBM;
        
        rssi_threshold_lookup_table[mcs] = dynamic_threshold;
    }
    
    printf("RSSI threshold lookup table built (MCS 0-9)\n");
}

// Build bitrate to MCS lookup table for 0-1000 Mbps
void build_bitrate_to_mcs_lookup_table(void) {
    printf("Building bitrate to MCS lookup table for 0-1000 Mbps\n");
    
    for (int bitrate = 0; bitrate <= 1000; bitrate++) {
        int mcs = 0;
        for (int i = 0; i < 10; i++) {
            if (bitrate <= BITRATE_MCS_THRESHOLDS[i]) {
                mcs = i;
                break;
            }
        }
        if (bitrate > BITRATE_MCS_THRESHOLDS[9]) {
            mcs = 9; // MCS 9 (highest)
        }
        bitrate_to_mcs_lookup_table[bitrate] = mcs;
    }
    
    printf("Bitrate to MCS lookup table built (0-1000 Mbps)\n");
}

// Initialize all performance lookup tables
void init_performance_lookup_tables(void) {
    printf("Initializing performance optimization lookup tables\n");
    
    build_exposure_lookup_table();
    build_mcs_command_lookup_table();
    build_rssi_threshold_lookup_table();
    build_bitrate_to_mcs_lookup_table();
    
    printf("All performance lookup tables initialized\n");
}


// Pre-calculated sleep value for performance (set during init)
static unsigned int sleep_value_us = 0; // Sleep value in microseconds

// Function pointer for sleep function (set once during init)
static void (*sleep_function)(bool, bool) = NULL;

// Function pointers for RSSI and dBm reading methods (set once during init)
static int (*rssi_read_function)(const char *) = NULL;
static int (*dbm_read_function)(const char *) = NULL;

// Function pointer for control algorithm (set once during init)
static int (*control_algorithm_function)(int, int, pid_controller_t*) = NULL;

// Function pointer for PIT mode operations (set once during init)
static void (*pit_mode_function)(void) = NULL;

// Function pointer for debug output (set once during init)
static void (*debug_output_function)(int, int, int) = NULL;

// Function pointer for cooldown logic (set once during init)
static bool (*cooldown_check_function)(int, int, unsigned long, unsigned long, int, unsigned long) = NULL;

// Debug output functions (optimized - no runtime conditionals)
static void debug_output_pid(int target_bitrate, int last_bitrate, int bitrate) {
    GLOBAL_DEBUG_BUILD(true, "PID: T=%d C=%d F=%d ", target_bitrate, last_bitrate, bitrate);
}

static void debug_output_fifo(int target_bitrate, int last_bitrate, int bitrate) {
    GLOBAL_DEBUG_BUILD(true, "FIFO: T=%d F=%d ", target_bitrate, bitrate);
}

// Initialize debug output function pointer (call once at startup)
void init_debug_output() {
    if (control_algorithm == CONTROL_ALGORITHM_PID) {
        debug_output_function = debug_output_pid;
    } else {
        debug_output_function = debug_output_fifo;
    }
}

// Asymmetric Cooldown Timing Variables (must be declared before cooldown functions)
static unsigned long last_change_time = 0;
static unsigned long last_up_time = 0;
static int last_bitrate = 0;

// Cooldown check functions (optimized - no runtime conditionals)
// These functions now handle ALL bitrate change logic including timing updates
static bool cooldown_check_disabled(int bitrate, int current_bitrate, unsigned long strict_cooldown_ms, 
                                   unsigned long up_cooldown_ms, int min_change_percent, unsigned long emergency_cooldown_ms) {
    (void)strict_cooldown_ms; (void)up_cooldown_ms; (void)min_change_percent; (void)emergency_cooldown_ms;
    
    // Always allow bitrate changes when cooldown is disabled
    // Update timing variables
    unsigned long now = get_current_time_ms();
    last_change_time = now;
    if (bitrate > current_bitrate) {
        last_up_time = now;
    }
    last_bitrate = bitrate;
    
    return true;
}

static bool cooldown_check_enabled(int bitrate, int current_bitrate, unsigned long strict_cooldown_ms, 
                                  unsigned long up_cooldown_ms, int min_change_percent, unsigned long emergency_cooldown_ms) {
    bool should_change = should_change_bitrate(bitrate, current_bitrate, strict_cooldown_ms, up_cooldown_ms, min_change_percent, emergency_cooldown_ms);
    
    // If cooldown check passes, update timing variables
    if (should_change) {
        unsigned long now = get_current_time_ms();
        last_change_time = now;
        if (bitrate > current_bitrate) {
            last_up_time = now;
        }
        last_bitrate = bitrate;
    }
    
    return should_change;
}

// Initialize cooldown check function pointer (call once at startup)
void init_cooldown_check() {
    if (cooldown_enabled == 0) {
        cooldown_check_function = cooldown_check_disabled;
        printf("Cooldown system: DISABLED (immediate bitrate changes)\n");
    } else {
        cooldown_check_function = cooldown_check_enabled;
        printf("Cooldown system: ENABLED (asymmetric timing)\n");
    }
}

// Initialize worker thread (call once at startup) - moved after variable declarations

// Counter-based sampling optimization functions
bool should_sample_signal_counter(void) {
    // Simple counter instead of expensive time calculations
    if (++sample_counter >= samples_per_loop) {  // Precomputed: 1000 / signal_sampling_freq_hz
        sample_counter = 0;
        return true;
    }
    return false;
}

void init_counter_sampling(void) {
    sample_counter = 0;
    samples_per_loop = 1000 / signal_sampling_freq_hz;  // Precompute division
    new_signal_data_available = false;
    printf("Counter-based sampling: Initialized for signal sampling (%lu samples per loop)\n", samples_per_loop);
}

// Initialize sleep value and function pointer (call once at startup)
void init_sleep_values() {
    // Set sleep value based on loop_timing_mode (never changes during runtime)
    if (loop_timing_mode == 0) {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.normal_mode_ms * 1000);
    } else if (loop_timing_mode == 1) {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.high_perf_mode_ms * 1000);
    } else if (loop_timing_mode == 2) {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.ultra_low_mode_ms * 1000);
    } else {
        sleep_value_us = sleep_config.main_loop_us + (sleep_config.normal_mode_ms * 1000); // Default
    }
    
    // Set function pointer based on sleep configuration (never changes during runtime)
    if (sleep_config.smart_sleep_enabled) {
        sleep_function = smart_sleep;
    } else {
        sleep_function = simple_sleep;
    }
    
    // Print sleep configuration
    printf("Sleep configuration:\n");
    printf("  Mode: %s\n", sleep_config.smart_sleep_enabled ? "Smart (adaptive)" : "Simple (fixed)");
    printf("  Base loop: %uus\n", sleep_config.main_loop_us);
    printf("  Sleep value: %uus (%.1fms)\n", sleep_value_us, sleep_value_us / 1000.0);
    printf("  Error conditions: %ums\n", sleep_config.error_condition_ms);
}


// Simple sleep function - just sleep based on pre-calculated value
void simple_sleep(bool is_error_condition, bool has_work_done) {
    (void)has_work_done; // Unused parameter for compatibility
    if (is_error_condition) {
        // Error conditions (RSSI=0, driver unavailable) - always sleep
        usleep(sleep_config.error_condition_ms * 1000);
        return;
    }
    
    // Use pre-calculated sleep value
    usleep(sleep_value_us);
}

// Smart sleep function - only sleep when we need to yield CPU
void smart_sleep(bool is_error_condition, bool has_work_done) {
    if (is_error_condition) {
        // Error conditions (RSSI=0, driver unavailable) - always sleep
        usleep(sleep_config.error_condition_ms * 1000);
        return;
    }
    
    // If we just did significant work (signal processing), sleep less
    if (has_work_done) {
        // Just yield CPU briefly - we've been busy
        usleep(sleep_config.main_loop_us); // Only base sleep
        return;
    }
    
    // If we're idle (no signal processing), sleep longer to save CPU
    usleep(sleep_value_us);
}

// Main loop sleep function - uses pre-calculated function pointer for maximum performance
void main_loop_sleep(bool is_error_condition, bool has_work_done) {
    sleep_function(is_error_condition, has_work_done);
}

// Generic popen() helper function to replace system() calls
int execute_command(const char *command) {
    // validate input parameters
    if (command == NULL) {
#ifdef DEBUG
        fprintf(stderr, "Error: execute_command() called with NULL command\n");
#endif
        return -1;
    }
    
    if (strlen(command) == 0) {
#ifdef DEBUG
        fprintf(stderr, "Error: execute_command() called with empty command\n");
#endif
        return -1;
    }
    
    if (strlen(command) > MAX_COMMAND_LENGTH) {
#ifdef DEBUG
        fprintf(stderr, "Error: execute_command() command too long (>%d chars)\n", MAX_COMMAND_LENGTH);
#endif
        return -1;
    }
    
    FILE *pipe = popen(command, "w");
    if (pipe == NULL) {
#ifdef DEBUG
        perror("popen failed");
#endif
        return -1;
    }
    
    int status = pclose(pipe);
    if (status == -1) {
#ifdef DEBUG
        perror("pclose failed");
#endif
        return -1;
    }
    // Return the exit status of the command
    return WEXITSTATUS(status);
}

// Filter instances table - organized by filter type
typedef struct {
    void* rssi_instance;
    void* dbm_instance;
} filter_instances_t;

// Global filter instances organized by type
static kalman_filter_t rssi_kalman_filter = {
    .estimate = 50.0f,           // Initial RSSI estimate (50%) - good starting point
    .error_estimate = 1.0f,      // Initial error estimate - safe default
    .process_variance = 0.0f,    // Default - updated from kalman_rssi_process config
    .measurement_variance = 0.0f // Default - updated from kalman_rssi_measure config
};

static kalman_filter_t dbm_kalman_filter = {
    .estimate = -60.0f,          // Initial dBm estimate - typical FPV range
    .error_estimate = 1.0f,      // Initial error estimate - safe default
    .process_variance = 0.0f,    // Default - updated from kalman_dbm_process config
    .measurement_variance = 0.0f // Default - updated from kalman_dbm_measure config
};

static lowpass_filter_t rssi_lowpass_filter = {
    .output = 50.0f,             // Initial RSSI estimate - good starting point
    .alpha = DEFAULT_FILTER_ALPHA,              // Default filter coefficient - moderate smoothing
    .initialised = false,        // Not initialized yet - will be set during init
    .cutoff_freq = DEFAULT_CUTOFF_FREQ,         // Default 2Hz cutoff - updated from config
    .sample_freq = DEFAULT_SAMPLE_FREQ         // Default 10Hz sample rate - updated from config
};

static lowpass_filter_t dbm_lowpass_filter = {
    .output = -60.0f,            // Initial dBm estimate - typical FPV range
    .alpha = DEFAULT_FILTER_ALPHA,              // Default filter coefficient - moderate smoothing
    .initialised = false,        // Not initialized yet - will be set during init
    .cutoff_freq = DEFAULT_CUTOFF_FREQ,         // Default 2Hz cutoff - updated from config
    .sample_freq = DEFAULT_SAMPLE_FREQ         // Default 10Hz sample rate - updated from config
};

static mode_filter_t rssi_mode_filter = {
    .sample_index = 0,            // Start at first sample position
    .return_element = 0,          // Default - updated from DEFAULT_MODE_FILTER_RETURN_ELEMENT
    .drop_high_sample = true,     // Start by dropping high samples (median-like behavior)
    .output = 50.0f               // Initial RSSI estimate - good starting point
};

static mode_filter_t dbm_mode_filter = {
    .sample_index = 0,            // Start at first sample position
    .return_element = 0,          // Default - updated from DEFAULT_MODE_FILTER_RETURN_ELEMENT
    .drop_high_sample = true,     // Start by dropping high samples (median-like behavior)
    .output = -60.0f              // Initial dBm estimate - typical FPV range
};

static derivative_filter_t rssi_derivative_filter = {
    .sample_index = 0,            // Start at first sample position
    .last_slope = 0.0f,          // No slope calculated yet
    .new_data = false             // No new data available yet
};

static derivative_filter_t dbm_derivative_filter = {
    .sample_index = 0,            // Start at first sample position
    .last_slope = 0.0f,          // No slope calculated yet
    .new_data = false             // No new data available yet
};

static lpf_2pole_t rssi_2pole_lpf = {
    .delay_element_1 = 0.0f,      // First delay element - starts at zero
    .delay_element_2 = 0.0f,      // Second delay element - starts at zero
    .cutoff_freq = 2.0f,          // Default 2Hz cutoff - updated from config
    .sample_freq = 10.0f,         // Default 10Hz sample rate - updated from config
    .initialised = false,         // Not initialized yet - will be set during init
    .output = 50.0f               // Initial RSSI estimate - good starting point
};

static lpf_2pole_t dbm_2pole_lpf = {
    .delay_element_1 = 0.0f,      // First delay element - starts at zero
    .delay_element_2 = 0.0f,      // Second delay element - starts at zero
    .cutoff_freq = 2.0f,          // Default 2Hz cutoff - updated from config
    .sample_freq = 10.0f,         // Default 10Hz sample rate - updated from config
    .initialised = false,         // Not initialized yet - will be set during init
    .output = -60.0f              // Initial dBm estimate - typical FPV range
};

static mean_filter_t rssi_mean_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,           // No samples collected yet
    .sum = 0.0f,                 // Running sum starts at zero
    .output = 50.0f              // Initial RSSI estimate - good starting point
};

static mean_filter_t dbm_mean_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,            // No samples collected yet
    .sum = 0.0f,                  // Running sum starts at zero
    .output = -60.0f             // Initial dBm estimate - typical FPV range
};

static gaussian_filter_t rssi_gaussian_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,            // No samples collected yet
    .output = 50.0f,             // Initial RSSI estimate - good starting point
    .weights_initialized = false  // Weights not calculated yet - will be done on first use
};

static gaussian_filter_t dbm_gaussian_filter = {
    .sample_index = 0,            // Start at first sample position
    .sample_count = 0,            // No samples collected yet
    .output = -60.0f,            // Initial dBm estimate - typical FPV range
    .weights_initialized = false  // Weights not calculated yet - will be done on first use
};

// Filter instances lookup table
static const filter_instances_t filter_instances_table[] = {
    [FILTER_TYPE_KALMAN] = {
        .rssi_instance = (void*)&rssi_kalman_filter,
        .dbm_instance = (void*)&dbm_kalman_filter
    },
    [FILTER_TYPE_LOWPASS] = {
        .rssi_instance = (void*)&rssi_lowpass_filter,
        .dbm_instance = (void*)&dbm_lowpass_filter
    },
    [FILTER_TYPE_MODE] = {
        .rssi_instance = (void*)&rssi_mode_filter,
        .dbm_instance = (void*)&dbm_mode_filter
    },
    [FILTER_TYPE_DERIVATIVE] = {
        .rssi_instance = (void*)&rssi_derivative_filter,
        .dbm_instance = (void*)&dbm_derivative_filter
    },
    [FILTER_TYPE_2POLE_LPF] = {
        .rssi_instance = (void*)&rssi_2pole_lpf,
        .dbm_instance = (void*)&dbm_2pole_lpf
    },
    [FILTER_TYPE_MEAN] = {
        .rssi_instance = (void*)&rssi_mean_filter,
        .dbm_instance = (void*)&dbm_mean_filter
    },
    [FILTER_TYPE_GAUSSIAN] = {
        .rssi_instance = (void*)&rssi_gaussian_filter,
        .dbm_instance = (void*)&dbm_gaussian_filter
    }
};

// Pre-calculated constant for filter instances table size (never changes at runtime)
static const int FILTER_INSTANCES_TABLE_SIZE = sizeof(filter_instances_table) / sizeof(filter_instances_table[0]);

// Global variable for RSSI reading method
int use_file_rewind_method = 0;  // 0 = mmap (default), 1 = file rewind

// Global WiFi driver availability flag
int wifi_driver_available = 1;  // Assume available until proven otherwise

// Initialize RSSI and dBm reading method function pointers (call once at startup)
void init_rssi_read_method() {
    printf("DEBUG: init_rssi_read_method called, use_file_rewind_method=%d\n", use_file_rewind_method);
    
    // Set function pointers based on RSSI read method configuration (applies to BOTH RSSI and dBm)
    if (use_file_rewind_method) {
        rssi_read_function = get_rssi_file_rewind;
        dbm_read_function = get_dbm_file_rewind;
        printf("DEBUG: Using file rewind method for RSSI and dBm\n");
    } else {
        rssi_read_function = get_rssi_mmap;
        dbm_read_function = get_dbm_mmap;
        printf("DEBUG: Using mmap method for RSSI and dBm\n");
    }
    
    
    // Print RSSI reading method optimization status
    if (use_file_rewind_method) {
        printf("RSSI/dBm reading optimization: File rewind with persistent handle\n");
    } else {
        printf("RSSI/dBm reading optimization: Memory mapping (mmap)\n");
    }
}


// Helper function to set PIT mode function pointer using lookup table
static void set_pit_mode_function(void) {
    const wifi_card_config_t* config = get_wifi_card_config(wificard);
    if (!config) {
        printf("PIT mode: ERROR - Unknown WiFi card type: %s\n", wificard);
        return;
    }
    
    pit_mode_function = pit_mode_enabled ? config->pit_mode_enable_func : config->pit_mode_disable_func;
}

// Initialize PIT mode function pointer (call once at startup)
void init_pit_mode() {
    set_pit_mode_function();
    
        if (pit_mode_enabled) {
        printf("PIT mode: %s ENABLED (low power standby)\n", wificard);
        } else {
        printf("PIT mode: %s DISABLED (full power FPV)\n", wificard);
    }
}

// Initialize race mode configuration
void init_race_mode() {
    if (race_mode != 1 && race_mode != 0) {
        printf("ERROR: Invalid value for race_mode: %d (must be 0 or 1)\n", race_mode);
        return;
    }
    
    if (race_mode == 1) {
        printf("RACEMODE ENABLED - Configuring for racing performance\n");
        
        // Set ACK timeout for racing (lower latency)
        char cmd1[512];
        snprintf(cmd1, sizeof(cmd1), "echo 20 > %s/ack_timeout", driverpath);
        int ack_result = execute_command(cmd1);
        if (ack_result != 0) {
            printf("WARNING: Could not set ack_timeout (driver may be protected)\n");
        } else {
            printf("Racing mode: ACK timeout set to 20ms\n");
        }
        
        // Override bitrate max for racing (4 Mbps)
        bitrate_max = 4;
        printf("Racing mode: Bitrate max set to %d Mbps\n", bitrate_max);
        
        
        } else {
        printf("Normal mode ENABLED - Configuring for stability\n");
    }
}

// Initialize HTTP system and video configuration
void init_http_system() {
    printf("HTTP system: Initializing video configuration\n");
    
    if (race_mode == 1) {
        // Configure racing video settings
        char video_config[256];
        snprintf(video_config, sizeof(video_config), "/api/v1/set?video0.size=%s", racing_video_resolution);
        http_get(video_config);
        
        snprintf(video_config, sizeof(video_config), "/api/v1/set?video0.fps=%d", racing_fps);
        http_get(video_config);
        
        // Calculate optimal exposure based on frame rate (180° shutter rule)
        int calculated_exposure = auto_exposure_enabled ? calculate_racing_exposure() : racing_exposure;
        snprintf(video_config, sizeof(video_config), "/api/v1/set?isp.exposure=%d", calculated_exposure);
        http_get(video_config);
        
        if (auto_exposure_enabled) {
            printf("HTTP: Racing mode video configured - %s@%dfps, exposure=%dms\n", 
                   racing_video_resolution, racing_fps, calculated_exposure);
        }
        
    } else {
        // Set normal mode video configuration
        char video_config[256];
        snprintf(video_config, sizeof(video_config), "/api/v1/set?video0.fps=%d", fps);
        http_get(video_config);
        
        // Calculate optimal exposure based on frame rate (180° shutter rule)
        int calculated_exposure = auto_exposure_enabled ? calculate_normal_exposure() : 8; // Default fallback
        snprintf(video_config, sizeof(video_config), "/api/v1/set?isp.exposure=%d", calculated_exposure);
        http_get(video_config);
        
        if (auto_exposure_enabled) {
            printf("HTTP: Normal mode video configured - %dfps, exposure=%dms\n", 
                   fps, calculated_exposure);
        }
    }

    
    printf("HTTP system: Video configuration complete\n");
}

// Global sleep configuration (not static - needs to persist and be accessible)
sleep_config_t sleep_config = {
    .main_loop_us = 100,        // 0.1ms base loop
    .normal_mode_ms = 20,       // +20ms for normal mode
    .high_perf_mode_ms = 5,     // +5ms for high performance
    .ultra_low_mode_ms = 0,     // No extra sleep for ultra-low latency
    .error_condition_ms = 10,   // 10ms for error conditions (was 100ms)
    .smart_sleep_enabled = 0    // Default: simple sleep (off)
};

// Global PID controller instance - will be initialized with static variables in main
static pid_controller_t bitrate_pid = {
    .kp = 0.0f,         // Will be set from pid_kp
    .ki = 0.0f,         // Will be set from pid_ki
    .kd = 0.0f,         // Will be set from pid_kd
    .integral = 0.0f,   // Start with no integral accumulation
    .last_error = 0,    // No previous error
    .last_output = 0    // No previous output
};


// Initialize control algorithm function pointer (call once at startup)
void init_control_algorithm() {
    // Set function pointer based on control algorithm configuration (never changes during runtime)
    if (control_algorithm == CONTROL_ALGORITHM_PID) {
        control_algorithm_function = pid_control_algorithm;
        // Initialize PID controller only when PID algorithm is selected
        pid_init(&bitrate_pid, pid_kp, pid_ki, pid_kd);
        printf("Control algorithm: PID Controller (smooth transitions)\n");
    } else {
        control_algorithm_function = fifo_control_algorithm;
        printf("Control algorithm: Simple FIFO (fast, direct)\n");
    }
}

// ArduPilot-style Low-Pass Filter Functions
// Calculate alpha coefficient for low-pass filter
float calc_lowpass_alpha_dt(float dt, float cutoff_freq) {
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        return 1.0f;  // No filtering
    }
    
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    return dt / (rc + dt);
}

// ArduPilot-style Low-Pass Filter Functions
void init_lowpass_filter(lowpass_filter_t *filter, float cutoff_freq, float sample_freq) {
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;
    filter->alpha = calc_lowpass_alpha_dt(1.0f / sample_freq, cutoff_freq);
    filter->initialised = false;
#ifdef DEBUG
    printf("Low-pass filter initialized: cutoff=%.1fHz, sample=%.1fHz, alpha=%.3f\n", 
           cutoff_freq, sample_freq, filter->alpha);
#endif
}

// Gaussian Filter Functions (Gaussian-weighted moving average)
// Initialize Gaussian weights (pre-calculated for efficiency)
void init_gaussian_weights(gaussian_filter_t *filter) {
    // Pre-calculate Gaussian weights for 7-sample window
    // Using global GAUSSIAN_SIGMA for runtime configurable sigma
    float sigma = GAUSSIAN_SIGMA;
    float center = (GAUSSIAN_FILTER_SIZE - 1) / 2.0f;
    
    float weight_sum = 0.0f;
    for (int i = 0; i < GAUSSIAN_FILTER_SIZE; i++) {
        float x = i - center;
        filter->weights[i] = expf(-(x * x) / (2.0f * sigma * sigma));
        weight_sum += filter->weights[i];
    }
    
    // Normalize weights so they sum to 1.0
    for (int i = 0; i < GAUSSIAN_FILTER_SIZE; i++) {
        filter->weights[i] /= weight_sum;
    }
    
    filter->weights_initialized = true;  // Mark weights as initialized
    
#ifdef DEBUG
    printf("Gaussian weights initialized (sigma=%.1f): ", sigma);
    for (int i = 0; i < GAUSSIAN_FILTER_SIZE; i++) {
        printf("%.3f ", filter->weights[i]);
    }
    printf("\n");
#endif
}


// 2-Pole Low-Pass Filter Functions (biquad filter)
// Initialize 2-pole low-pass filter
void init_2pole_lpf(lpf_2pole_t *filter, float cutoff_freq, float sample_freq) {
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;
    
    // Pre-calculate filter coefficients (expensive operations done once)
    float omega = 2.0f * M_PI * cutoff_freq / sample_freq;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * LPF_2POLE_Q_FACTOR);
    
    // Calculate biquad coefficients
    float b0_raw = (1.0f - cs) / 2.0f;
    float b1_raw = 1.0f - cs;
    float b2_raw = (1.0f - cs) / 2.0f;
    float a0_raw = 1.0f + alpha;
    float a1_raw = -2.0f * cs;
    float a2_raw = 1.0f - alpha;
    
    // Normalize coefficients (a0 becomes 1.0)
    filter->b0 = b0_raw / a0_raw;
    filter->b1 = b1_raw / a0_raw;
    filter->b2 = b2_raw / a0_raw;
    filter->a1 = a1_raw / a0_raw;
    filter->a2 = a2_raw / a0_raw;
    
    // Initialize delay elements
    filter->delay_element_1 = 0.0f;
    filter->delay_element_2 = 0.0f;
    filter->initialised = false;
    
#ifdef DEBUG
    printf("2-Pole LPF initialized: cutoff=%.1fHz, sample=%.1fHz\n", cutoff_freq, sample_freq);
    printf("Pre-calculated coefficients: b0=%.3f, b1=%.3f, b2=%.3f, a1=%.3f, a2=%.3f\n",
           filter->b0, filter->b1, filter->b2, filter->a1, filter->a2);
#endif
}

// Individual Filter Processing Functions (for lookup table)
static float process_kalman_filter(float sample, void* filter_data, bool is_rssi) {
    kalman_filter_t *filter = (kalman_filter_t*)filter_data;
                
                // Prediction step - predict next state
                float predicted_estimate = filter->estimate;
                float predicted_error = filter->error_estimate + filter->process_variance;
                
                // Update step - correct prediction with measurement
                float kalman_gain = predicted_error / (predicted_error + filter->measurement_variance);
                
                // Clamp kalman gain to prevent numerical issues
                if (kalman_gain > 1.0f) kalman_gain = 1.0f;
                if (kalman_gain < 0.0f) kalman_gain = 0.0f;
                
    filter->estimate = predicted_estimate + kalman_gain * (sample - predicted_estimate);
                filter->error_estimate = (1.0f - kalman_gain) * predicted_error;
                
                // Prevent error estimate from becoming too small (numerical stability)
                if (filter->error_estimate < KALMAN_MIN_ERROR_ESTIMATE) {
                    filter->error_estimate = KALMAN_MIN_ERROR_ESTIMATE;
    }
    
    return filter->estimate;
}

static float process_lowpass_filter(float sample, void* filter_data, bool is_rssi) {
    lowpass_filter_t *filter = (lowpass_filter_t*)filter_data;
    filter->output += (sample - filter->output) * filter->alpha;
    return filter->output;
}

static float process_mode_filter(float sample, void* filter_data, bool is_rssi) {
    mode_filter_t *filter = (mode_filter_t*)filter_data;
                
                // Insertion sort for mode filter (alternates dropping high/low samples)
                uint8_t j;
                
                // If buffer isn't full, simply increase sample count
                if (filter->sample_index < MODE_FILTER_SIZE) {
                    filter->sample_index++;
                    filter->drop_high_sample = true;  // Default to dropping high
                }
                
                if (filter->drop_high_sample) {
                    // Drop highest sample - start from top
                    j = filter->sample_index - 1;
                    
                    // Shift samples down to make room
        while (j > 0 && filter->samples[j-1] > sample) {
                        filter->samples[j] = filter->samples[j-1];
                        j--;
                    }
                    
                    // Insert new sample
        filter->samples[j] = sample;
                } else {
                    // Drop lowest sample - start from bottom
                    j = 0;
                    
                    // Shift samples up to make room
        while (j < filter->sample_index - 1 && filter->samples[j+1] < sample) {
                        filter->samples[j] = filter->samples[j+1];
                        j++;
                    }
                    
                    // Insert new sample
        filter->samples[j] = sample;
                }
                
                // Alternate drop direction for next sample
                filter->drop_high_sample = !filter->drop_high_sample;
                
                if (filter->sample_index < MODE_FILTER_SIZE) {
                    // Buffer not full - return middle sample
                    filter->output = filter->samples[filter->sample_index / 2];
                } else {
                    // Buffer full - return specified element (usually median)
                    filter->output = filter->samples[filter->return_element];
                }
                
    return filter->output;
            }
                
static float process_derivative_filter(float sample, void* filter_data, bool is_rssi) {
    derivative_filter_t *filter = (derivative_filter_t*)filter_data;
                
                uint32_t current_time = (uint32_t)(get_current_time_ms());
                uint8_t j = filter->sample_index;
                uint8_t j1;
                
                if (j == 0) {
                    j1 = DERIVATIVE_FILTER_SIZE - 1;
                } else {
                    j1 = j - 1;
                }
                
                // Check if this is a new timestamp
                if (filter->timestamps[j1] == current_time) {
        return sample; // Ignore duplicate timestamp
                }
                
                // Store timestamp and sample
                filter->timestamps[j] = current_time;
    filter->samples[j] = sample;
                
                // Update sample index
                filter->sample_index = (filter->sample_index + 1) % DERIVATIVE_FILTER_SIZE;
                filter->new_data = true;
                
                // Return the most recent sample (derivative is available via slope function)
                uint8_t idx = (filter->sample_index - 1 + DERIVATIVE_FILTER_SIZE) % DERIVATIVE_FILTER_SIZE;
    return filter->samples[idx];
            }
                
static float process_2pole_lpf_filter(float sample, void* filter_data, bool is_rssi) {
    lpf_2pole_t *filter = (lpf_2pole_t*)filter_data;
                
                // Apply biquad filter using pre-calculated coefficients (no expensive trig functions!)
    float output = filter->b0 * sample + filter->b1 * filter->delay_element_1 + filter->b2 * filter->delay_element_2
                               - filter->a1 * filter->delay_element_1 - filter->a2 * filter->delay_element_2;
                
                // Update delay elements
                filter->delay_element_2 = filter->delay_element_1;
                filter->delay_element_1 = output;
                
                filter->output = output;
    return output;
            }
                
static float process_mean_filter(float sample, void* filter_data, bool is_rssi) {
    mean_filter_t *filter = (mean_filter_t*)filter_data;
                
                // If buffer isn't full yet, just add to sum
                if (filter->sample_count < MEAN_FILTER_SIZE) {
        filter->samples[filter->sample_count] = sample;
        filter->sum += sample;
                    filter->sample_count++;
                    filter->output = filter->sum / filter->sample_count;
        return filter->output;
                }
                
                // Buffer is full, use circular buffer
                // Remove oldest sample from sum
                filter->sum -= filter->samples[filter->sample_index];
                
                // Add new sample
    filter->samples[filter->sample_index] = sample;
    filter->sum += sample;
                
                // Calculate mean
                filter->output = filter->sum / MEAN_FILTER_SIZE;
                
                // Update circular buffer index
                filter->sample_index = (filter->sample_index + 1) % MEAN_FILTER_SIZE;
                
    return filter->output;
            }
                
static float process_gaussian_filter(float sample, void* filter_data, bool is_rssi) {
    gaussian_filter_t *filter = (gaussian_filter_t*)filter_data;
                
                // If buffer isn't full yet, just add to buffer
                if (filter->sample_count < GAUSSIAN_FILTER_SIZE) {
        filter->samples[filter->sample_count] = sample;
                    filter->sample_count++;
                    
                    // Calculate weighted average with available samples
                    float weighted_sum = 0.0f;
                    float weight_sum = 0.0f;
                    for (int j = 0; j < filter->sample_count; j++) {
                        weighted_sum += filter->samples[j] * filter->weights[j];
                        weight_sum += filter->weights[j];
                    }
                    filter->output = weighted_sum / weight_sum;
        return filter->output;
                }
                
                // Buffer is full, use circular buffer
                // Store new sample at current index
    filter->samples[filter->sample_index] = sample;
                
                // Calculate Gaussian-weighted average using pre-calculated weights
                float weighted_sum = 0.0f;
                for (int j = 0; j < GAUSSIAN_FILTER_SIZE; j++) {
                    weighted_sum += filter->samples[j] * filter->weights[j];
                }
                filter->output = weighted_sum; // Weights are already normalized
                
                // Update circular buffer index
                filter->sample_index = (filter->sample_index + 1) % GAUSSIAN_FILTER_SIZE;
                
    return filter->output;
}

// Filter initialization functions
static void init_kalman_filter(void* filter_data, bool is_rssi) {
    // Kalman filters are initialized elsewhere with process/measurement variance
    (void)filter_data;  // Suppress unused parameter warning
    (void)is_rssi;     // Suppress unused parameter warning
}

static void init_lowpass_filter_wrapper(void* filter_data, bool is_rssi) {
    lowpass_filter_t *filter = (lowpass_filter_t*)filter_data;
    filter->output = DEFAULT_RSSI_ESTIMATE;
    filter->initialised = false;
}

static void init_mode_filter_wrapper(void* filter_data, bool is_rssi) {
    mode_filter_t *filter = (mode_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->drop_high_sample = true;
    filter->return_element = DEFAULT_MODE_FILTER_RETURN_ELEMENT;
}

static void init_derivative_filter_wrapper(void* filter_data, bool is_rssi) {
    derivative_filter_t *filter = (derivative_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->new_data = false;
}

static void init_2pole_lpf_filter_wrapper(void* filter_data, bool is_rssi) {
    lpf_2pole_t *filter = (lpf_2pole_t*)filter_data;
    filter->output = DEFAULT_RSSI_ESTIMATE;
    filter->delay_element_1 = DEFAULT_RSSI_ESTIMATE;
    filter->delay_element_2 = DEFAULT_RSSI_ESTIMATE;
    filter->initialised = false;
}

static void init_mean_filter_wrapper(void* filter_data, bool is_rssi) {
    mean_filter_t *filter = (mean_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->sample_count = 0;
    filter->sum = 0.0f;
    filter->output = DEFAULT_RSSI_ESTIMATE;
}

static void init_gaussian_filter_wrapper(void* filter_data, bool is_rssi) {
    gaussian_filter_t *filter = (gaussian_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->sample_count = 0;
    filter->output = DEFAULT_RSSI_ESTIMATE;
    init_gaussian_weights(filter);
}

// Filter reset functions
static void reset_kalman_filter(void* filter_data, bool is_rssi) {
    kalman_filter_t *filter = (kalman_filter_t*)filter_data;
    filter->estimate = is_rssi ? DEFAULT_RSSI_ESTIMATE : DEFAULT_DBM_ESTIMATE;
    filter->error_estimate = DEFAULT_ERROR_ESTIMATE;
}

static void reset_lowpass_filter(void* filter_data, bool is_rssi) {
    lowpass_filter_t *filter = (lowpass_filter_t*)filter_data;
    filter->output = is_rssi ? DEFAULT_RSSI_ESTIMATE : DEFAULT_DBM_ESTIMATE;
    filter->initialised = false;
}

static void reset_mode_filter(void* filter_data, bool is_rssi) {
    mode_filter_t *filter = (mode_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->drop_high_sample = true;
    filter->output = is_rssi ? DEFAULT_RSSI_ESTIMATE : DEFAULT_DBM_ESTIMATE;
}

static void reset_derivative_filter(void* filter_data, bool is_rssi) {
    derivative_filter_t *filter = (derivative_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->new_data = false;
}

static void reset_2pole_lpf_filter(void* filter_data, bool is_rssi) {
    lpf_2pole_t *filter = (lpf_2pole_t*)filter_data;
    float default_value = is_rssi ? DEFAULT_RSSI_ESTIMATE : DEFAULT_DBM_ESTIMATE;
    filter->output = default_value;
    filter->delay_element_1 = default_value;
    filter->delay_element_2 = default_value;
    filter->initialised = false;
}

static void reset_mean_filter(void* filter_data, bool is_rssi) {
    mean_filter_t *filter = (mean_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->sample_count = 0;
    filter->sum = 0.0f;
    filter->output = is_rssi ? DEFAULT_RSSI_ESTIMATE : DEFAULT_DBM_ESTIMATE;
}

static void reset_gaussian_filter(void* filter_data, bool is_rssi) {
    gaussian_filter_t *filter = (gaussian_filter_t*)filter_data;
    filter->sample_index = 0;
    filter->sample_count = 0;
    filter->output = is_rssi ? DEFAULT_RSSI_ESTIMATE : DEFAULT_DBM_ESTIMATE;
}

// Filter lookup table - maps filter types to their processing functions
static const filter_lookup_entry_t filter_lookup_table[] = {
    [FILTER_TYPE_KALMAN] = {
        .process_func = process_kalman_filter,
        .init_func = init_kalman_filter,
        .reset_func = reset_kalman_filter,
        .name = "Kalman"
    },
    [FILTER_TYPE_LOWPASS] = {
        .process_func = process_lowpass_filter,
        .init_func = init_lowpass_filter_wrapper,
        .reset_func = reset_lowpass_filter,
        .name = "Low-Pass"
    },
    [FILTER_TYPE_MODE] = {
        .process_func = process_mode_filter,
        .init_func = init_mode_filter_wrapper,
        .reset_func = reset_mode_filter,
        .name = "Mode"
    },
    [FILTER_TYPE_DERIVATIVE] = {
        .process_func = process_derivative_filter,
        .init_func = init_derivative_filter_wrapper,
        .reset_func = reset_derivative_filter,
        .name = "Derivative"
    },
    [FILTER_TYPE_2POLE_LPF] = {
        .process_func = process_2pole_lpf_filter,
        .init_func = init_2pole_lpf_filter_wrapper,
        .reset_func = reset_2pole_lpf_filter,
        .name = "2-Pole LPF"
    },
    [FILTER_TYPE_MEAN] = {
        .process_func = process_mean_filter,
        .init_func = init_mean_filter_wrapper,
        .reset_func = reset_mean_filter,
        .name = "Mean"
    },
    [FILTER_TYPE_GAUSSIAN] = {
        .process_func = process_gaussian_filter,
        .init_func = init_gaussian_filter_wrapper,
        .reset_func = reset_gaussian_filter,
        .name = "Gaussian"
    }
};

// Pre-calculated constant for filter lookup table size (never changes at runtime)
static const int FILTER_LOOKUP_TABLE_SIZE = sizeof(filter_lookup_table) / sizeof(filter_lookup_table[0]);

// Helper function to get filter data pointer using instances lookup table
static void* get_filter_data(filter_chain_t *chain, filter_type_t filter_type) {
    // Bounds check using pre-calculated constant
    if (filter_type >= FILTER_INSTANCES_TABLE_SIZE) {
        return NULL;
    }
    
    const filter_instances_t *instances = &filter_instances_table[filter_type];
    bool is_rssi = (chain == &rssi_filter_chain);
    
    return is_rssi ? instances->rssi_instance : instances->dbm_instance;
}

// Filter Chain Functions
// Apply a complete filter chain to a sample (optimized with lookup tables)
float apply_filter_chain(filter_chain_t *chain, float sample) {
    if (!chain->enabled || chain->filter_count == 0) {
        return sample;
    }
    
    float filtered_sample = sample;
    
#ifdef DEBUG
    // Centralized error checking and logging
    char error_message[512] = {0};
    char log_buffer[1024] = {0};
    bool has_errors = false;
    bool has_warnings = false;
#endif
    
    // Pre-initialize all filters that need initialization using lookup table
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        // Bounds check for lookup table
        if (filter_type >= FILTER_LOOKUP_TABLE_SIZE) {
#ifdef DEBUG
                has_errors = true;
                snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message), 
                     "Invalid filter type %d, ", filter_type);
#endif
            continue;
        }
        
        const filter_lookup_entry_t *entry = &filter_lookup_table[filter_type];
        void* filter_data = get_filter_data(chain, filter_type);
        
        if (!filter_data || !entry->init_func) {
#ifdef DEBUG
            has_errors = true;
            snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message), 
                     "Missing filter data or init function for type %d, ", filter_type);
#endif
            continue;
        }
        
        // Special initialization logic for filters that need it
        if (filter_type == FILTER_TYPE_LOWPASS) {
            lowpass_filter_t *filter = (lowpass_filter_t*)filter_data;
            if (!filter->initialised) {
                filter->output = filtered_sample;
                filter->initialised = true;
#ifdef DEBUG
                snprintf(log_buffer + strlen(log_buffer), sizeof(log_buffer) - strlen(log_buffer), 
                         "Low-Pass initialized (%.1f), ", filtered_sample);
#endif
            }
        } else if (filter_type == FILTER_TYPE_2POLE_LPF) {
            lpf_2pole_t *filter = (lpf_2pole_t*)filter_data;
            if (!filter->initialised) {
                filter->output = filtered_sample;
                filter->delay_element_1 = filtered_sample;
                filter->delay_element_2 = filtered_sample;
                filter->initialised = true;
#ifdef DEBUG
                snprintf(log_buffer + strlen(log_buffer), sizeof(log_buffer) - strlen(log_buffer), 
                         "2-Pole LPF initialized (%.1f), ", filtered_sample);
#endif
            }
        } else if (filter_type == FILTER_TYPE_GAUSSIAN) {
            gaussian_filter_t *filter = (gaussian_filter_t*)filter_data;
            if (!filter->weights_initialized) {
                init_gaussian_weights(filter);
#ifdef DEBUG
                snprintf(log_buffer + strlen(log_buffer), sizeof(log_buffer) - strlen(log_buffer), 
                         "Gaussian weights initialized, ");
#endif
            }
        }
    }
    
    // Apply each filter in the chain sequentially using lookup table
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        // Bounds check for lookup table
        if (filter_type >= FILTER_LOOKUP_TABLE_SIZE) {
#ifdef DEBUG
            has_errors = true;
            snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message), 
                     "Invalid filter type %d, ", filter_type);
#endif
            continue;
        }
        
        const filter_lookup_entry_t *entry = &filter_lookup_table[filter_type];
        void* filter_data = get_filter_data(chain, filter_type);
        
        if (!filter_data || !entry->process_func) {
#ifdef DEBUG
            has_errors = true;
            snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message), 
                     "Missing filter data or process function for type %d, ", filter_type);
#endif
            continue;
        }
        
        // Process sample through filter using lookup table
        bool is_rssi = (chain == &rssi_filter_chain);
        filtered_sample = entry->process_func(filtered_sample, filter_data, is_rssi);
    }
    
#ifdef DEBUG
    // Output consolidated debug information only if there are issues or initialization
    if (has_errors || has_warnings || strlen(log_buffer) > 0) {
        GLOBAL_DEBUG_BUILD(true, "Filter Chain Debug: ");
        if (strlen(log_buffer) > 0) {
            GLOBAL_DEBUG_APPEND("%s", log_buffer);
        }
        if (has_errors || has_warnings) {
            GLOBAL_DEBUG_APPEND("Issues: %s", error_message);
        }
        GLOBAL_DEBUG_APPEND("Input: %.1f -> Output: %.1f ", sample, filtered_sample);
    }
#endif
    
    return filtered_sample;
}

// Set filter chain configuration from string (e.g., "2,0" for Mode->Kalman)
void set_filter_chain(const char *config_str, filter_chain_t *chain) {
    if (!config_str || strlen(config_str) == 0) {
        // Default to single Kalman filter
        chain->filters[0] = FILTER_TYPE_KALMAN;
        chain->filter_count = 1;
        return;
    }
    
    char *config_copy = strdup(config_str);
    char *token = strtok(config_copy, ",");
    uint8_t count = 0;
    
    while (token != NULL && count < MAX_FILTERS_PER_CHAIN) {
        int filter_type = atoi(token);
        if (filter_type >= 0 && filter_type <= 6) {
            chain->filters[count] = (filter_type_t)filter_type;
            count++;
        }
        token = strtok(NULL, ",");
    }
    
    chain->filter_count = count;
    free(config_copy);
    
    // Print filter chain configuration using lookup table
#ifdef DEBUG
    printf("Filter chain configured: ");
    for (uint8_t i = 0; i < count; i++) {
        filter_type_t filter_type = chain->filters[i];
        const char *filter_name = "Unknown";
        
        // Use lookup table for filter names
        if (filter_type < FILTER_LOOKUP_TABLE_SIZE) {
            filter_name = filter_lookup_table[filter_type].name;
        }
        
        printf("%s", filter_name);
        if (i < count - 1) printf(" -> ");
    }
    printf(" (%d filters)\n", count);
#endif
}

// Reset all filters in a chain using lookup table
void reset_filter_chain(filter_chain_t *chain) {
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        // Bounds check for lookup table
        if (filter_type >= FILTER_LOOKUP_TABLE_SIZE) {
            continue; // Skip invalid filter types
        }
        
        const filter_lookup_entry_t *entry = &filter_lookup_table[filter_type];
        void* filter_data = get_filter_data(chain, filter_type);
        
        if (!filter_data || !entry->reset_func) {
            continue; // Skip if no data or reset function
        }
        
        // Reset filter using lookup table
        bool is_rssi = (chain == &rssi_filter_chain);
        entry->reset_func(filter_data, is_rssi);
    }
}

// Toggle between racing and normal mode filter chains
void toggle_racemode(void) {
    race_mode = !race_mode;  // Toggle race mode
    
    if (race_mode) {
        // Switch to racing mode filters
        active_rssi_filter_chain = &rssi_race_filter_chain;
        active_dbm_filter_chain = &dbm_race_filter_chain;
        printf("Racing mode ENABLED - Using low-pass filters for fast response\n");
    } else {
        // Switch to normal mode filters
        active_rssi_filter_chain = &rssi_filter_chain;
        active_dbm_filter_chain = &dbm_filter_chain;
        printf("Normal mode ENABLED - Using Kalman filters for stability\n");
    }
}

// Initialize filters and set up filter chains based on race mode
void init_filters(float rssi_process_var, float rssi_measure_var,
                 float dbm_process_var, float dbm_measure_var,
                 float lpf_cutoff_freq, float lpf_sample_freq) {
    
    // Set and configure filter chains based on race_mode
    if (race_mode) {
        // Racing mode - only initialize racing filters
        set_filter_chain(racing_rssi_filter_chain_config, &rssi_race_filter_chain);
        set_filter_chain(racing_dbm_filter_chain_config, &dbm_race_filter_chain);
        active_rssi_filter_chain = &rssi_race_filter_chain;
        active_dbm_filter_chain = &dbm_race_filter_chain;
        printf("Racing mode ENABLED - Using low-pass filters for fast response\n");
    } else {
        // Normal mode - only initialize normal filters
        set_filter_chain(rssi_filter_chain_config, &rssi_filter_chain);
        set_filter_chain(dbm_filter_chain_config, &dbm_filter_chain);
        active_rssi_filter_chain = &rssi_filter_chain;
        active_dbm_filter_chain = &dbm_filter_chain;
        printf("Normal mode ENABLED - Using Kalman filters for stability\n");
    }
    
    // Only initialize filters if the active filter chains are properly set
    if (active_rssi_filter_chain && active_dbm_filter_chain) {
        // Initialize filters based on active filter chains
        init_filter_chain_filters(active_rssi_filter_chain, rssi_process_var, rssi_measure_var, 
                                  lpf_cutoff_freq, lpf_sample_freq, true);
        init_filter_chain_filters(active_dbm_filter_chain, dbm_process_var, dbm_measure_var, 
                                  lpf_cutoff_freq, lpf_sample_freq, false);
        
#ifdef DEBUG
        printf("Active filter chains initialized successfully\n");
#endif
    } else {
        printf("WARNING: Active filter chains not set - skipping filter initialization\n");
    }
}

// Initialize filters for a specific filter chain (optimized for performance)
void init_filter_chain_filters(filter_chain_t *chain, float process_var, float measure_var,
                               float lpf_cutoff_freq, float lpf_sample_freq, bool is_rssi) {
    // Early exit if chain is not enabled or has no filters
    if (!chain || !chain->enabled || chain->filter_count == 0) {
        return;
    }
    
    // Only initialize the specific filter types that are configured
    for (uint8_t i = 0; i < chain->filter_count; i++) {
        filter_type_t filter_type = chain->filters[i];
        
        switch (filter_type) {
            case FILTER_TYPE_LOWPASS: {
                lowpass_filter_t *filter = is_rssi ? &rssi_lowpass_filter : &dbm_lowpass_filter;
                init_lowpass_filter(filter, lpf_cutoff_freq, lpf_sample_freq);
#ifdef DEBUG
                printf("Low-pass filter initialized for %s\n", is_rssi ? "RSSI" : "dBm");
#endif
                break;
            }
            case FILTER_TYPE_KALMAN: {
                kalman_filter_t *filter = is_rssi ? &rssi_kalman_filter : &dbm_kalman_filter;
                filter->process_variance = process_var;
                filter->measurement_variance = measure_var;
                filter->estimate = is_rssi ? DEFAULT_RSSI_ESTIMATE : DEFAULT_DBM_ESTIMATE;
                filter->error_estimate = DEFAULT_ERROR_ESTIMATE;
#ifdef DEBUG
                printf("Kalman filter initialized for %s\n", is_rssi ? "RSSI" : "dBm");
#endif
                break;
            }
    
    
            case FILTER_TYPE_MODE: {
                mode_filter_t *filter = is_rssi ? &rssi_mode_filter : &dbm_mode_filter;
                filter->return_element = DEFAULT_MODE_FILTER_RETURN_ELEMENT;
#ifdef DEBUG
                printf("Mode filter initialized for %s\n", is_rssi ? "RSSI" : "dBm");
#endif
                break;
            }
            case FILTER_TYPE_DERIVATIVE: {
                // Derivative filters are initialized by default
#ifdef DEBUG
                printf("Derivative filter initialized for %s\n", is_rssi ? "RSSI" : "dBm");
#endif
                break;
            }
            
            case FILTER_TYPE_2POLE_LPF: {
                lpf_2pole_t *filter = is_rssi ? &rssi_2pole_lpf : &dbm_2pole_lpf;
                init_2pole_lpf(filter, lpf_cutoff_freq, lpf_sample_freq);
#ifdef DEBUG
                printf("2-Pole LPF initialized for %s\n", is_rssi ? "RSSI" : "dBm");
#endif
                break;
            }
    
            case FILTER_TYPE_MEAN: {
                // Mean filters are initialized by default
#ifdef DEBUG
                printf("Mean filter initialized for %s\n", is_rssi ? "RSSI" : "dBm");
#endif
                break;
            }
            
            case FILTER_TYPE_GAUSSIAN: {
                gaussian_filter_t *filter = is_rssi ? &rssi_gaussian_filter : &dbm_gaussian_filter;
                init_gaussian_weights(filter);
#ifdef DEBUG
                printf("Gaussian filter initialized for %s\n", is_rssi ? "RSSI" : "dBm");
#endif
                break;
            }
        }
    }
}

// Reset all filters
void reset_filters() {
    // Reset filter chains
    reset_filter_chain(&rssi_filter_chain);
    reset_filter_chain(&dbm_filter_chain);
    
#ifdef DEBUG
    printf("All filter chains reset\n");
#endif
}



// Get current time in milliseconds
unsigned long get_current_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

// PID Controller Functions
// Calculate PID output based on target and current values
int pid_calculate(pid_controller_t *pid, int target, int current) {
    // Calculate error
    int error = target - current;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term (with windup protection)
    pid->integral += error;
    
    // Integral windup protection - limit integral to prevent overshoot
    const float max_integral = 1000.0f;  // Adjust based on your bitrate range
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    int derivative = error - pid->last_error;
    float d_term = pid->kd * derivative;
    
    // Calculate PID output
    float pid_output = p_term + i_term + d_term;
    
    // Update PID state
    pid->last_error = error;
    
    // Return the adjustment (not absolute value)
    return (int)pid_output;
}

// Reset PID controller state
void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->last_error = 0;
    pid->last_output = 0;
#ifdef DEBUG
    printf("PID controller reset\n");
#endif
}

// Control Algorithm Functions
// PID Controller: Smooth transitions with PID control
int pid_control_algorithm(int target_bitrate, int last_bitrate, pid_controller_t *pid) {
    int pid_adjustment = pid_calculate(pid, target_bitrate, last_bitrate);
    return last_bitrate + pid_adjustment;
}

// Simple FIFO: Direct assignment (faster, more responsive)
int fifo_control_algorithm(int target_bitrate, int last_bitrate, pid_controller_t *pid) {
    return target_bitrate;  // Direct assignment
}

// Initialize PID controller with custom parameters
void pid_init(pid_controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid_reset(pid);
    
    printf("PID Controller initialized: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
}

// Check if bitrate change should be allowed based on asymmetric cooldown
bool should_change_bitrate(int new_bitrate, int current_bitrate, 
                          unsigned long strict_cooldown_ms, unsigned long up_cooldown_ms,
                          int min_change_percent, unsigned long emergency_cooldown_ms) {
    unsigned long now = get_current_time_ms();
    int delta = abs(new_bitrate - current_bitrate);
    int min_delta = current_bitrate * min_change_percent / 100;
    
    // Don't change if delta is too small
    if (delta < min_delta) {
        return false;
    }
    
    if (new_bitrate < current_bitrate) {
        // Can decrease quickly - use emergency cooldown for faster response
        return (now - last_change_time) >= emergency_cooldown_ms;
    } else {
        // Must wait longer to increase - need both strict and up cooldown
        return (now - last_change_time) >= strict_cooldown_ms &&
               (now - last_up_time) >= up_cooldown_ms;
    }
}

// Emergency drop logic for sudden signal loss
void check_emergency_drop(int current_bitrate, float filtered_rssi, 
                         int emergency_rssi_threshold, int emergency_bitrate) {
    // Calculate dynamic RSSI threshold based on current MCS
    int current_mcs = bitrate_to_mcs(current_bitrate);
    int dynamic_threshold = get_dynamic_rssi_threshold(current_mcs);
    
    // Use the more conservative threshold (lower value = more sensitive)
    int effective_threshold = (dynamic_threshold < emergency_rssi_threshold) ? 
                             dynamic_threshold : emergency_rssi_threshold;
    
    if (filtered_rssi < effective_threshold && current_bitrate > emergency_bitrate) {
#ifdef DEBUG
        printf("EMERGENCY DROP: RSSI=%.1f (threshold=%d, MCS=%d), bitrate=%d->%d\n", 
               filtered_rssi, effective_threshold, current_mcs, current_bitrate, emergency_bitrate);
#endif
        
        // Set emergency bitrate
        char command[COMMAND_BUFFER_SIZE];
        snprintf(command, sizeof(command), "wfb_tx_cmd 8000 set_bitrate %d", emergency_bitrate);
        int result = execute_command(command);
        if (result != 0) {
#ifdef DEBUG
            printf("Warning: Emergency bitrate command failed with status %d\n", result);
#endif
        }
        
        // Update timing variables
        unsigned long now = get_current_time_ms();
        last_change_time = now;
        last_up_time = now;
        last_bitrate = emergency_bitrate;
        
        // Reset all filters and PID controller to adapt to new conditions
        reset_filters();
        if (control_algorithm == CONTROL_ALGORITHM_PID) {
        pid_reset(&bitrate_pid);
        }
        
  
    }
}


// Set WiFi transmission power to maximum for maximum range
// Critical for FPV applications where range is essential
void set_maximum_tx_power(void) {
    if (!enable_maximum_tx_power) {
        printf("Maximum TX power is DISABLED - This may limit your FPV range!\n");
        printf("Enable enable_maximum_tx_power=1 in config for maximum range\n");
        return;
    }
    
    // Get WiFi card configuration from lookup table
    const wifi_card_config_t* config = get_wifi_card_config(wificard);
    if (!config) {
        printf("ERROR: Unknown WiFi card type for TX power: %s\n", wificard);
        return;
    }
    
    // Execute maximum TX power command from lookup table
    printf("Setting maximum TX power for %s...\n", config->power_save_description);
    int result = execute_command(config->max_tx_power_command);
    
    if (result != 0) {
        printf("Warning: Failed to set maximum TX power (status %d)\n", result);
        printf("Command attempted: %s\n", config->max_tx_power_command);
        printf("This may limit your FPV range\n");
    } else {
        printf("Maximum TX power SET - Optimized for maximum FPV range\n");
    }
}

// Setup driver-level power management for both RTL8812AU and RTL8812EU
// This provides additional protection beyond iw commands
void setup_driver_power_management(void) {
    if (!disable_wifi_power_save) {
        printf("Driver power management: ENABLED (not recommended for FPV)\n");
        return;
    }
    
    // Get WiFi card configuration from lookup table
    const wifi_card_config_t* config = get_wifi_card_config(wificard);
    if (!config) {
        printf("ERROR: Unknown WiFi card type for driver power management: %s\n", wificard);
        return;
    }
    
    printf("Setting up %s driver power management...\n", config->power_save_description);
    
    // Execute driver reload command from lookup table
    int result = execute_command(config->driver_reload_command);
        
    if (result != 0) {
        printf("Warning: Failed to reload %s driver with power management disabled\n", config->name);
        printf("You may need to manually add to /etc/modprobe.d/%s.conf:\n", config->name);
        printf("%s\n", config->modprobe_config_message);
    } else {
        printf("%s driver reloaded with power management DISABLED\n", config->name);
    }
    
    // Execute power save command to disable power saving
    printf("Disabling WiFi power saving via iw command...\n");
    int power_result = execute_command(config->power_save_command);
    if (power_result != 0) {
        printf("Warning: Failed to disable power saving via iw command\n");
    } else {
        printf("WiFi power saving DISABLED via iw command\n");
    }
}

// Enable PIT mode: Low power standby with HTTP wake-up capability
// Perfect for battery conservation during racing events or standby periods
void enable_pit_mode(void) {
    printf("PIT MODE: ENABLED - Low power standby with HTTP wake-up\n");
    printf("System will reduce TX power for battery conservation but remain responsive to HTTP calls\n");
    
    // Get WiFi card configuration from lookup table
    const wifi_card_config_t* config = get_wifi_card_config(wificard);
    if (!config) {
        printf("PIT MODE: ERROR - Unknown WiFi card type: %s\n", wificard);
        return;
    }
    
    // Execute PIT mode TX power command from lookup table
    printf("PIT MODE: Reducing TX power for %s (battery conservation)...\n", config->power_save_description);
    int result = execute_command(config->pit_tx_power_command);
    
    if (result == 0) {
        printf("PIT MODE: TX power REDUCED for battery conservation\n");
        printf("PIT MODE: System ready for HTTP wake-up calls\n");
    } else {
        printf("PIT MODE: Warning - Failed to reduce TX power\n");
    }
}

// Disable PIT mode: Return to full power FPV mode
// Called when exiting standby or when HTTP wake-up is received
void disable_pit_mode(void) {
    printf("PIT MODE: DISABLING - Returning to full power FPV mode\n");
    
    // Get WiFi card configuration from lookup table
    const wifi_card_config_t* config = get_wifi_card_config(wificard);
    if (!config) {
        printf("PIT MODE: ERROR - Unknown WiFi card type: %s\n", wificard);
        return;
    }
    
    // Execute maximum TX power command from lookup table
    printf("PIT MODE: Restoring maximum TX power for %s...\n", config->power_save_description);
    int result = execute_command(config->max_tx_power_command);
    
    if (result == 0) {
        printf("PIT MODE: Maximum TX power RESTORED for full FPV performance\n");
        printf("PIT MODE: System ready for full power FPV operation\n");
    } else {
        printf("PIT MODE: Warning - Failed to restore maximum TX power\n");
    }
}

// PIT Mode Function Pointer Functions - Card Specific
// RTL8812AU PIT mode functions
void pit_mode_enable_8812au(void) {
    printf("PIT MODE: ENABLED - Low power standby with HTTP wake-up\n");
    printf("System will reduce TX power for battery conservation but remain responsive to HTTP calls\n");
    
    char command[128];
    snprintf(command, sizeof(command), "iw dev wlan0 set txpower fixed 10");
    printf("PIT MODE: Reducing TX power for RTL8812AU to 10 dBm (battery conservation)...\n");
    
    int result = execute_command(command);
    if (result == 0) {
        printf("PIT MODE: TX power REDUCED for battery conservation\n");
        printf("PIT MODE: System ready for HTTP wake-up calls\n");
    } else {
        printf("PIT MODE: Warning - Failed to reduce TX power\n");
    }
}

void pit_mode_disable_8812au(void) {
    printf("PIT MODE: DISABLING - Returning to full power FPV mode\n");
    
    char command[128];
    snprintf(command, sizeof(command), "iw dev wlan0 set txpower fixed 30");
    printf("PIT MODE: Restoring maximum TX power for RTL8812AU to 30 dBm...\n");
    
    int result = execute_command(command);
    if (result == 0) {
        printf("PIT MODE: Maximum TX power RESTORED for full FPV performance\n");
        printf("PIT MODE: System ready for full power FPV operation\n");
    } else {
        printf("PIT MODE: Warning - Failed to restore maximum TX power\n");
    }
}

// RTL8812EU PIT mode functions
void pit_mode_enable_8812eu2(void) {
    printf("PIT MODE: ENABLED - Low power standby with HTTP wake-up\n");
    printf("System will reduce TX power for battery conservation but remain responsive to HTTP calls\n");
    
    char command[128];
    snprintf(command, sizeof(command), "iw dev wlan0 set txpower fixed 10");
    printf("PIT MODE: Reducing TX power for RTL8812EU to 10 dBm (battery conservation)...\n");
    
    int result = execute_command(command);
    if (result == 0) {
        printf("PIT MODE: TX power REDUCED for battery conservation\n");
        printf("PIT MODE: System ready for HTTP wake-up calls\n");
    } else {
        printf("PIT MODE: Warning - Failed to reduce TX power\n");
    }
}

void pit_mode_disable_8812eu2(void) {
    printf("PIT MODE: DISABLING - Returning to full power FPV mode\n");
    
    char command[128];
    snprintf(command, sizeof(command), "iw dev wlan0 set txpower fixed 30");
    printf("PIT MODE: Restoring maximum TX power for RTL8812EU to 30 dBm...\n");
    
    int result = execute_command(command);
    if (result == 0) {
        printf("PIT MODE: Maximum TX power RESTORED for full FPV performance\n");
        printf("PIT MODE: System ready for full power FPV operation\n");
    } else {
        printf("PIT MODE: Warning - Failed to restore maximum TX power\n");
    }
}

// RTL873xBU PIT mode functions
void pit_mode_enable_873xbu(void) {
    printf("PIT MODE: ENABLED - Low power standby with HTTP wake-up\n");
    printf("System will reduce TX power for battery conservation but remain responsive to HTTP calls\n");
    
    char command[128];
    snprintf(command, sizeof(command), "iw dev wlan0 set txpower fixed 5");
    printf("PIT MODE: Reducing TX power for RTL873xBU to 5 dBm (battery conservation)...\n");
    
    int result = execute_command(command);
    if (result == 0) {
        printf("PIT MODE: TX power REDUCED for battery conservation\n");
        printf("PIT MODE: System ready for HTTP wake-up calls\n");
    } else {
        printf("PIT MODE: Warning - Failed to reduce TX power\n");
    }
}

void pit_mode_disable_873xbu(void) {
    printf("PIT MODE: DISABLING - Returning to full power FPV mode\n");
    
    char command[128];
    snprintf(command, sizeof(command), "iw dev wlan0 set txpower fixed 20");
    printf("PIT MODE: Restoring maximum TX power for RTL873xBU to 20 dBm...\n");
    
    int result = execute_command(command);
    if (result == 0) {
        printf("PIT MODE: Maximum TX power RESTORED for full FPV performance\n");
        printf("PIT MODE: System ready for full power FPV operation\n");
    } else {
        printf("PIT MODE: Warning - Failed to restore maximum TX power\n");
    }
}

// Simple toggle function for PIT mode (no HTTP required)
// Can be called directly for testing or manual control
void toggle_pit_mode(void) {
    // Toggle PIT mode state
    pit_mode_enabled = !pit_mode_enabled;
    
    // Update function pointer using helper
    set_pit_mode_function();
    
    // Execute the appropriate function
    pit_mode_function();
    
    printf("PIT MODE: %s %s via toggle\n", wificard, pit_mode_enabled ? "ENABLED" : "DISABLED");
}

// HTTP API endpoint for toggling PIT mode
// GET /api/toggle/pitmode - Toggle between PIT mode and full power
int toggle_pit_mode_http(void) {
    char url[256];
    snprintf(url, sizeof(url), "http://localhost:8080/api/toggle/pitmode");
    
    int result = http_get(url);
    if (result == 0) {
        // Toggle PIT mode state
        pit_mode_enabled = !pit_mode_enabled;
        
        // Update function pointer using helper
        set_pit_mode_function();
        
        // Execute the appropriate function
        pit_mode_function();
        
        printf("HTTP API: PIT MODE %s %s via HTTP call\n", wificard, pit_mode_enabled ? "ENABLED" : "DISABLED");
        
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "HTTP: PIT mode %s ", pit_mode_enabled ? "ENABLED" : "DISABLED");
#endif
        
        return 0;
    } else {
        printf("HTTP API: Failed to toggle PIT mode (HTTP call failed)\n");
        return -1;
    }
}

// Set real-time priority for ultra-high performance racing VTX
int set_realtime_priority() {
    struct sched_param param;
    
    // Set low real-time priority (reduced to prevent SSH issues)
    param.sched_priority = REALTIME_PRIORITY;  // Low priority (1-99 range)
    
    // Use SCHED_FIFO for consistent timing
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("Failed to set real-time priority");
        printf("Note: Real-time priority requires root privileges\n");
        printf("Run with: sudo ./ap_alink\n");
        return -1;
    }
    
    printf("Real-time priority set successfully (SCHED_FIFO, priority 10)\n");
    return 0;
}


// Optimized HTTP GET implementation - fire-and-forget for FPV applications
int http_get(const char *path) {
    int s;
    struct sockaddr_in addr;
    char req[256];
    // Fast timeout since we don't wait for responses
    struct timeval tv = { .tv_sec = 0, .tv_usec = HTTP_TIMEOUT_US }; // 100ms timeout

    if ((s = socket(AF_INET, SOCK_STREAM, 0)) < 0) return -1;

    // Only set send timeout since we don't receive responses
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    addr.sin_family = AF_INET;
    addr.sin_port = htons(HTTP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(s);
        return -1;
    }

    snprintf(req, sizeof(req), "GET %s HTTP/1.0\r\n\r\n", path);
    if (send(s, req, strlen(req), 0) < 0) {
        close(s);
        return -1;
    }

    // Close immediately - no need to wait for response in FPV applications
    close(s);
    return 0;
}

// Runtime Configuration API - Simple HTTP-based configuration updates
// Usage: http_get("/api/config/signal?high_rssi=60&medium_rssi=45&low_rssi=25")
//        http_get("/api/config/pid?kp=1.2&ki=0.1&kd=0.5")
//        http_get("/api/config/filter?rssi_est=55&dbm_est=-65&alpha=0.15")
//        http_get("/api/config/system?safety=5&timeout=150000&priority=15")
int update_runtime_config(const char *config_type, const char *params) {
    char api_path[512];
    
    // Build API path for configuration update
    snprintf(api_path, sizeof(api_path), "/api/config/%s?%s", config_type, params);
    
    // Send HTTP request to update configuration
    int result = http_get(api_path);
    
#ifdef DEBUG
    if (result == 0) {
        printf("Runtime config updated: %s with params: %s\n", config_type, params);
    } else {
        printf("Failed to update runtime config: %s\n", config_type);
    }
#endif
    
    return result;
}

// HTTP API function to toggle race mode
// Usage: Call this function to toggle between racing and normal mode filters
// HTTP endpoint: /api/toggle/racemode
int toggle_racemode_http(void) {
    char api_path[512];
    
    // Build API path for race mode toggle
    snprintf(api_path, sizeof(api_path), "/api/toggle/racemode");
    
    // Send HTTP request to toggle race mode
    int result = http_get(api_path);
    
    if (result == 0) {
        // HTTP request successful, toggle race mode locally
        toggle_racemode();
        printf("Race mode toggled via HTTP API\n");
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "HTTP: Race mode toggled successfully ");
#endif
    } else {
        printf("Failed to toggle race mode via HTTP API\n");
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "HTTP: Race mode toggle failed ");
#endif
    }
    
    return result;
}

// Worker thread infrastructure
static pthread_t worker_thread;
static pthread_mutex_t worker_mutex = PTHREAD_MUTEX_INITIALIZER;
static sem_t worker_sem;
static worker_cmd_t pending_cmd;
static int worker_running = 0;

// QP Delta configuration removed - ROI is disabled in recent Majestic builds
// Video quality is now controlled by MCS rate control only


// Calculate dynamic RSSI threshold based on current MCS - Uses lookup table for performance
int get_dynamic_rssi_threshold(int current_mcs) {
    // Use lookup table for performance
    if (current_mcs >= 0 && current_mcs < 10) {
        return rssi_threshold_lookup_table[current_mcs];
    }
    
    // Clamp MCS to valid range and use lookup table
    if (current_mcs < 0) current_mcs = 0;
    if (current_mcs >= 10) current_mcs = 9;
    
    return rssi_threshold_lookup_table[current_mcs];
}

// Convert bitrate to approximate MCS - Uses lookup table for performance
int bitrate_to_mcs(int bitrate_mbps) {
    // Use lookup table for performance
    if (bitrate_mbps >= 0 && bitrate_mbps <= 1000) {
        return bitrate_to_mcs_lookup_table[bitrate_mbps];
    }
    
    // Fallback for out-of-range values
    if (bitrate_mbps <= 0) return 0;
    if (bitrate_mbps > 1000) return 9;
    
    // Mapping based on actual MCS data rates for 20MHz
    for (int i = 0; i < 10; i++) {
        if (bitrate_mbps <= BITRATE_MCS_THRESHOLDS[i]) {
            return i;
        }
    }
    return 9; // MCS 9 (highest)
}

// =============================================================================
// PURE FUNCTIONS - No side effects, same input = same output
// =============================================================================

// Pure function versions for better testability and optimization
int bitrate_to_mcs_pure(int bitrate_mbps, const double* thresholds) {
    for (int i = 0; i < 10; i++) {
        if (bitrate_mbps <= thresholds[i]) {
            return i;
        }
    }
    return 9; // MCS 9 (highest)
}

int get_dynamic_rssi_threshold_pure(int current_mcs, 
                                   const int* mcs_thresholds,
                                   int hardware_rssi_offset,
                                   int hardware_mcs_offset) {
    // Clamp MCS to valid range
    if (current_mcs < 0) current_mcs = 0;
    if (current_mcs >= 10) current_mcs = 9;
    
    // Get base threshold from lookup table
    int base_threshold = mcs_thresholds[current_mcs];
    
    // Apply hardware-specific offset
    int dynamic_threshold = base_threshold + hardware_rssi_offset;
    
    // Apply hardware-specific MCS offset
    dynamic_threshold += hardware_mcs_offset;
    
    // Add safety margin (3 dBm) for emergency drop
    return dynamic_threshold + 3;
}

// Pure VLQ calculation (no side effects)
double calculate_vlq_pure(int filtered_dbm, int dbm_min, int dbm_max, int filtered_rssi) {
    if (filtered_dbm < -100 || filtered_dbm > 0) {
        // Invalid dBm reading, use RSSI fallback
        return (filtered_rssi > 0) ? filtered_rssi : 0.0;
    }
    
    if (dbm_max == dbm_min) {
        // Fallback: use RSSI-based calculation
        return (filtered_rssi > 0) ? filtered_rssi : 0.0;
    }
    
    // Calculate VLQ using precomputed range inverse
    double dbm_range_inv = 100.0 / (double)(dbm_max - dbm_min);
    return (filtered_dbm - dbm_min) * dbm_range_inv;
}

// Pure bitrate calculation
int calculate_target_bitrate_pure(double vlq, int bitrate_max) {
    return (int)(vlq * bitrate_max / 100.0);
}

// Pure exposure calculation
int calculate_exposure_pure(int fps) {
    if (fps <= 0) return 8;
    
    float frame_period_ms = 1000.0f / fps;
    float exposure_ms = frame_period_ms / 2.0f;
    
    int exposure = (int)(exposure_ms + 0.5f);
    return (exposure < 1) ? 1 : exposure;
}


// Pure MCS command selection - Uses lookup table for performance
int get_mcs_command_pure(int mcs_index, const char* wificard_type) {
    // Use lookup table for performance
    if (mcs_index >= 0 && mcs_index < 16) {
        return mcs_command_lookup_table[mcs_index];
    }
    
    return 0xFF;  // Auto rate adaptation
}

// Initialize worker thread (call once at startup)
void init_worker_thread() {
    sem_init(&worker_sem, 0, 0);
    worker_running = 1;
    if (pthread_create(&worker_thread, NULL, worker_thread_func, NULL) != 0) {
        perror("Failed to create worker thread");
        exit(1);
    }
    printf("Worker thread: Initialized and started\n");
}

// Worker thread that processes API calls
void *worker_thread_func(void *arg) {
    char cmd[512];
    
    while (worker_running) {
        // Wait for work
        sem_wait(&worker_sem);
        
        pthread_mutex_lock(&worker_mutex);
        worker_cmd_t cmd_to_process = pending_cmd;
        pthread_mutex_unlock(&worker_mutex);
        
        switch (cmd_to_process.type) {
            case CMD_SET_BITRATE: {
                int bitrate_kbps = cmd_to_process.data.bitrate_kbps;
                char path[128];
                snprintf(path, sizeof(path), "/api/v1/set?video0.bitrate=%d", bitrate_kbps);
                http_get(path);
                break;
            }
            case CMD_SET_MCS: {
                mcs_arg_t *data = &cmd_to_process.data.mcs_data;
                int bitrateMcs = data->bitrateMcs;
                char *mcspath = data->mcspath;
                
                // Hardware-specific MCS rate control (using proper Realtek MCS encoding)
                int mcs_command = get_mcs_command_pure(bitrateMcs, wificard);
                snprintf(cmd, sizeof(cmd), "echo 0x%02x > %s/rate_ctl", mcs_command, mcspath);
                
                    int result = execute_command(cmd);
                    if (result != 0) {
#ifdef DEBUG
                    printf("Warning: MCS rate control (MCS=%d, cmd=0x%02x) command failed with status %d\n", 
                           bitrateMcs, mcs_command, result);
#endif
                } else {
#ifdef DEBUG
                    GLOBAL_DEBUG_BUILD(true, "MCS rate control set: MCS=%d, Command=%s ", bitrateMcs, cmd);
#endif
                }
                break;
            }
        }
    }
    return NULL;
}



void set_mcs_async(int bitrateMcs, const char *mcspath) {
    if (!worker_running) return;
    
    pthread_mutex_lock(&worker_mutex);
    pending_cmd.type = CMD_SET_MCS;
    pending_cmd.data.mcs_data.bitrateMcs = bitrateMcs;
    // Safe string copy with bounds checking
    strncpy(pending_cmd.data.mcs_data.mcspath, mcspath, sizeof(pending_cmd.data.mcs_data.mcspath) - 1);
    pending_cmd.data.mcs_data.mcspath[sizeof(pending_cmd.data.mcs_data.mcspath) - 1] = '\0';
    pthread_mutex_unlock(&worker_mutex);
    
    sem_post(&worker_sem);
}






void config(const char *filename, int *BITRATE_MAX, char *WIFICARD, int *RACE, int *FPS,
             float *rssi_process_var, float *rssi_measure_var,
             float *dbm_process_var, float *dbm_measure_var,
             unsigned long *strict_cooldown, unsigned long *up_cooldown,
             int *min_change_percent, int *emergency_rssi_threshold, int *emergency_bitrate,
             float *pid_kp, float *pid_ki, float *pid_kd,
             float *lpf_cutoff_freq, float *lpf_sample_freq,
             char *rssi_filter_chain_config, char *dbm_filter_chain_config,
             char *racing_rssi_filter_chain_config, char *racing_dbm_filter_chain_config,
             char *racing_video_resolution, int *racing_exposure, int *racing_fps,
             int *signal_sampling_interval, unsigned long *emergency_cooldown,
             int *control_algorithm, int *signal_sampling_freq_hz, int *hardware_rssi_offset, int *cooldown_enabled, int *auto_exposure_enabled) {
    FILE* fp = fopen(filename, "r");
    if (!fp) {
        printf("No config file found\n");
        exit(1);
    }

    char line[128];

    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "bitrate_max=", 12) == 0) {
            sscanf(line + 12, "%d", BITRATE_MAX);
            continue;
        }
        if (strncmp(line, "wificard=", 9) == 0) {
            sscanf(line + 9, "%63s", WIFICARD);
            continue;
        }
        if (strncmp(line, "race_mode=", 10) == 0) {
            sscanf(line + 10, "%d", RACE);
            continue;
        }
        if (strncmp(line, "fps=", 4) == 0) {
            sscanf(line + 4, "%d", FPS);
            continue;
        }
        if (strncmp(line, "kalman_rssi_process=", 20) == 0) {
            sscanf(line + 20, "%f", rssi_process_var);
            continue;
        }
        if (strncmp(line, "kalman_rssi_measure=", 20) == 0) {
            sscanf(line + 20, "%f", rssi_measure_var);
            continue;
        }
        if (strncmp(line, "kalman_dbm_process=", 19) == 0) {
            sscanf(line + 19, "%f", dbm_process_var);
            continue;
        }
        if (strncmp(line, "kalman_dbm_measure=", 19) == 0) {
            sscanf(line + 19, "%f", dbm_measure_var);
            continue;
        }
        if (strncmp(line, "strict_cooldown_ms=", 19) == 0) {
            sscanf(line + 19, "%lu", strict_cooldown);
            continue;
        }
        if (strncmp(line, "up_cooldown_ms=", 15) == 0) {
            sscanf(line + 15, "%lu", up_cooldown);
            continue;
        }
        if (strncmp(line, "min_change_percent=", 19) == 0) {
            sscanf(line + 19, "%d", min_change_percent);
            continue;
        }
        if (strncmp(line, "emergency_rssi_threshold=", 25) == 0) {
            sscanf(line + 25, "%d", emergency_rssi_threshold);
            continue;
        }
        if (strncmp(line, "emergency_bitrate=", 18) == 0) {
            sscanf(line + 18, "%d", emergency_bitrate);
            continue;
        }
        if (strncmp(line, "pid_kp=", 7) == 0) {
            sscanf(line + 7, "%f", pid_kp);
            continue;
        }
        if (strncmp(line, "pid_ki=", 7) == 0) {
            sscanf(line + 7, "%f", pid_ki);
            continue;
        }
        if (strncmp(line, "pid_kd=", 7) == 0) {
            sscanf(line + 7, "%f", pid_kd);
            continue;
        }
        if (strncmp(line, "rssi_filter_chain=", 18) == 0) {
            sscanf(line + 18, "%63s", rssi_filter_chain_config);
            continue;
        }
        if (strncmp(line, "dbm_filter_chain=", 17) == 0) {
            sscanf(line + 17, "%63s", dbm_filter_chain_config);
            continue;
        }
        if (strncmp(line, "racing_rssi_filter_chain=", 24) == 0) {
            sscanf(line + 24, "%63s", racing_rssi_filter_chain_config);
            continue;
        }
        if (strncmp(line, "racing_dbm_filter_chain=", 23) == 0) {
            sscanf(line + 23, "%63s", racing_dbm_filter_chain_config);
            continue;
        }
        if (strncmp(line, "racing_video_resolution=", 24) == 0) {
            sscanf(line + 24, "%63s", racing_video_resolution);
            continue;
        }
        if (strncmp(line, "racing_exposure=", 16) == 0) {
            sscanf(line + 16, "%d", racing_exposure);
            continue;
        }
        if (strncmp(line, "racing_fps=", 11) == 0) {
            sscanf(line + 11, "%d", racing_fps);
            continue;
        }
        if (strncmp(line, "lpf_cutoff_freq=", 16) == 0) {
            sscanf(line + 16, "%f", lpf_cutoff_freq);
            continue;
        }
        if (strncmp(line, "lpf_sample_freq=", 16) == 0) {
            sscanf(line + 16, "%f", lpf_sample_freq);
            continue;
        }
        if (strncmp(line, "signal_sampling_interval=", 26) == 0) {
            sscanf(line + 26, "%d", signal_sampling_interval);
            continue;
        }
        if (strncmp(line, "emergency_cooldown_ms=", 22) == 0) {
            sscanf(line + 22, "%lu", emergency_cooldown);
            continue;
        }
        if (strncmp(line, "control_algorithm=", 18) == 0) {
            sscanf(line + 18, "%d", control_algorithm);
            continue;
        }
        // QP Delta settings removed - ROI is disabled in recent Majestic builds
        if (strncmp(line, "signal_sampling_freq_hz=", 24) == 0) {
            sscanf(line + 24, "%d", signal_sampling_freq_hz);
            continue;
        }
        if (strncmp(line, "hardware_rssi_offset=", 21) == 0) {
            sscanf(line + 21, "%d", hardware_rssi_offset);
            continue;
        }
        if (strncmp(line, "cooldown_enabled=", 17) == 0) {
            sscanf(line + 17, "%d", cooldown_enabled);
            continue;
        }
        if (strncmp(line, "disable_wifi_power_save=", 23) == 0) {
            sscanf(line + 23, "%d", &disable_wifi_power_save);
            continue;
        }
        if (strncmp(line, "enable_maximum_tx_power=", 23) == 0) {
            sscanf(line + 23, "%d", &enable_maximum_tx_power);
            continue;
        }
        if (strncmp(line, "auto_exposure_enabled=", 22) == 0) {
            sscanf(line + 22, "%d", auto_exposure_enabled);
            continue;
        }
        if (strncmp(line, "pit_mode_enabled=", 17) == 0) {
            sscanf(line + 17, "%d", &pit_mode_enabled);
            continue;
        }
        if (strncmp(line, "rssi_read_method=", 17) == 0) {
            int method;
            sscanf(line + 17, "%d", &method);
            use_file_rewind_method = (method == 1);  // 1 = file rewind, 0 = mmap
            printf("DEBUG: Config parsed rssi_read_method=%d, use_file_rewind_method=%d\n", method, use_file_rewind_method);
            continue;
        }
        // Sleep configuration
        if (strncmp(line, "sleep_main_loop_us=", 19) == 0) {
            sscanf(line + 19, "%u", &sleep_config.main_loop_us);
            continue;
        }
        if (strncmp(line, "sleep_normal_mode_ms=", 21) == 0) {
            sscanf(line + 21, "%u", &sleep_config.normal_mode_ms);
            continue;
        }
        if (strncmp(line, "sleep_high_perf_mode_ms=", 24) == 0) {
            sscanf(line + 24, "%u", &sleep_config.high_perf_mode_ms);
            continue;
        }
        if (strncmp(line, "sleep_ultra_low_mode_ms=", 24) == 0) {
            sscanf(line + 24, "%u", &sleep_config.ultra_low_mode_ms);
            continue;
        }
        if (strncmp(line, "sleep_error_condition_ms=", 26) == 0) {
            sscanf(line + 26, "%u", &sleep_config.error_condition_ms);
            continue;
        }
        if (strncmp(line, "smart_sleep_enabled=", 20) == 0) {
            sscanf(line + 20, "%d", &sleep_config.smart_sleep_enabled);
            continue;
        }
        // Frame sync removed - not needed
    }

    fclose(fp);
}


// Optimized dBm reading wrapper - uses function pointer for performance
int get_dbm(const char *readcmd) {
    if (dbm_read_function == NULL) {
        printf("ERROR: dBm read function not initialized! Call init_rssi_read_method() first.\n");
        return -100;
    }
    return dbm_read_function(readcmd);
}



// Driver-specific format parsers (no more guessing!)
int parse_rssi_8812eu_format(const char *buffer) {
    // 8812EU driver format: "rssi : 85 %"
    char *pos = strstr(buffer, "rssi");
    if (pos) {
        int rssi_percent;
        if (sscanf(pos, "rssi : %d %%", &rssi_percent) == 1) {
            return rssi_percent;
        }
    }
    return 0;
}

int parse_rssi_8812au_format(const char *buffer) {
    // 8812AU driver format: "rssi: 85 %"
    char *pos = strstr(buffer, "rssi");
    if (pos) {
        int rssi_percent;
        if (sscanf(pos, "rssi: %d %%", &rssi_percent) == 1) {
            return rssi_percent;
        }
    }
    return 0;
}

int parse_rssi_873xbu_format(const char *buffer) {
    // 873xBU driver format: "signal: -45 dBm" (convert to percentage)
    char *pos = strstr(buffer, "signal");
    if (pos) {
        int signal_dbm;
        if (sscanf(pos, "signal: %d dBm", &signal_dbm) == 1) {
            // Convert dBm to percentage: -30 dBm = 100%, -90 dBm = 0%
            int rssi_percent = (signal_dbm + 90) * 100 / 60;
            if (rssi_percent > 100) rssi_percent = 100;
            if (rssi_percent < 0) rssi_percent = 0;
            return rssi_percent;
        }
    }
    return 0;
}

// Driver-specific dBm parsers (for backup method)
int parse_dbm_8812eu_format(const char *buffer) {
    // 8812EU driver format: "rssi : -45 dBm"
    char *pos = strstr(buffer, "rssi");
    if (pos) {
        int rssi_dbm;
        if (sscanf(pos, "rssi : %d dBm", &rssi_dbm) == 1) {
            return rssi_dbm;
        }
    }
    return -100;
}

int parse_dbm_8812au_format(const char *buffer) {
    // 8812AU driver format: "rssi: -45 dBm"
    char *pos = strstr(buffer, "rssi");
    if (pos) {
        int rssi_dbm;
        if (sscanf(pos, "rssi: %d dBm", &rssi_dbm) == 1) {
            return rssi_dbm;
        }
    }
    return -100;
}

int parse_dbm_873xbu_format(const char *buffer) {
    // 873xBU driver format: "signal: -45 dBm"
    char *pos = strstr(buffer, "signal");
    if (pos) {
        int signal_dbm;
        if (sscanf(pos, "signal: %d dBm", &signal_dbm) == 1) {
            return signal_dbm;
        }
    }
    return -100;
}

int get_dbm_from_sta_tp_info(const char *readcmd) {
    char path[512];
    int dbm = -100;
    
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");
    
    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        return dbm;
    }
    
    char buffer[1024];
    if (fread(buffer, 1, sizeof(buffer)-1, fp) > 0) {
        buffer[sizeof(buffer)-1] = '\0';
        
        // Use driver-specific dBm parser (no more guessing!)
        dbm = dbm_format_parser(buffer);
    }
    
    fclose(fp);
    return dbm;
}

// File read with rewind approach
int get_rssi_file_rewind(const char *readcmd) {
    static FILE *fp = NULL;
    static char last_path[512] = {0};
    char buffer[256];
    int rssi_percent = 0;
    char path[512];

    // Build the full path to sta_tp_info
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");

    // Only reopen if path changed or file not open
    if (fp == NULL || strcmp(path, last_path) != 0) {
        if (fp != NULL) {
            fclose(fp);
        }
        fp = fopen(path, "r");
        if (!fp) {
#ifdef DEBUG
            perror("fopen");
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to open sta_tp_info file: %s ", path);
#endif
            return -1;  // Return error code instead of 0 RSSI
        }
        strcpy(last_path, path);
    } else {
        // Rewind to beginning for fresh read
        rewind(fp);
    }

    while (fgets(buffer, sizeof(buffer), fp)) {
        // Use driver-specific format parser (no more guessing!)
        rssi_percent = rssi_format_parser(buffer);
        if (rssi_percent > 0) {
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Found RSSI via file rewind: %d%% ", rssi_percent);
#endif
            return rssi_percent;
        }
    }

    return rssi_percent;
}

// Memory mapped approach
int get_rssi_mmap(const char *readcmd) {
    static char *mapped_data = NULL;
    static size_t mapped_size = 0;
    static char last_path[512] = {0};
    char path[512];
    int rssi_percent = 0;
    
    // Direct string concatenation - much faster than snprintf
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");
    
    // Only remap if path changed
    if (strcmp(path, last_path) != 0) {
        if (mapped_data != NULL) {
            munmap(mapped_data, mapped_size);
            mapped_data = NULL;
        }
        
        int fd = open(path, O_RDONLY);
        if (fd < 0) {
            perror("open");
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to open sta_tp_info file: %s ", path);
#endif
            return -1;  // Return error code instead of 0 RSSI
        }
        
        struct stat st;
        if (fstat(fd, &st) < 0) {
            perror("fstat");
            close(fd);
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to stat sta_tp_info file: %s ", path);
#endif
            return -1;  // Return error code instead of 0 RSSI
        }
        
        mapped_size = st.st_size;
        if (mapped_size == 0) {
            close(fd);
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: No WiFi stations connected (sta_tp_info empty): %s ", path);
#endif
            return -1;  // Return error code instead of 0 RSSI
        }
        
        mapped_data = mmap(NULL, mapped_size, PROT_READ, MAP_PRIVATE, fd, 0);
        close(fd);
        
        if (mapped_data == MAP_FAILED) {
            perror("mmap");
            mapped_data = NULL;
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to mmap sta_tp_info file: %s ", path);
#endif
            return -1;  // Return error code instead of 0 RSSI
        }
        
        strcpy(last_path, path);
    }
    
    if (mapped_data == NULL) {
        return rssi_percent;
    }
    
    // Use driver-specific format parser (no more guessing!)
    rssi_percent = rssi_format_parser(mapped_data);
    if (rssi_percent > 0) {
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "DEBUG: Found RSSI in file: %d%% ", rssi_percent);
#endif
    } else {
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "DEBUG: No RSSI found in sta_tp_info ");
        GLOBAL_DEBUG_BUILD(true, "DEBUG: File content (first 200 chars): %.200s ", mapped_data);
#endif
    }
    
    return rssi_percent;
}

// Optimized wrapper function using function pointer (no runtime conditionals)
int get_rssi(const char *readcmd) {
    return rssi_read_function(readcmd);
}

// =============================================================================
// OPTIMIZED dBm READING FUNCTIONS (File Rewind and Memory Mapping)
// =============================================================================

// File read with rewind approach for dBm
int get_dbm_file_rewind(const char *readcmd) {
    static FILE *fp = NULL;
    static char last_path[512] = {0};
    char buffer[256];
    int dbm = -100;
    char path[512];

    // Build the full path to sta_tp_info
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");

    // Only reopen if path changed or file not open
    if (fp == NULL || strcmp(path, last_path) != 0) {
        if (fp != NULL) {
            fclose(fp);
        }
        fp = fopen(path, "r");
        if (!fp) {
#ifdef DEBUG
            perror("fopen");
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to open sta_tp_info file for dBm: %s ", path);
#endif
            return -100;  // Return error code
        }
        strcpy(last_path, path);
    } else {
        // Rewind to beginning for fresh read
        rewind(fp);
    }

    while (fgets(buffer, sizeof(buffer), fp)) {
        // Use driver-specific dBm format parser
        dbm = dbm_format_parser(buffer);
        if (dbm != -100 && dbm != -1) {  // Valid dBm value found
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Found dBm via file rewind: %d dBm ", dbm);
#endif
            return dbm;
        }
    }

    return dbm;
}

// Memory mapped approach for dBm
int get_dbm_mmap(const char *readcmd) {
    static char *mapped_data = NULL;
    static size_t mapped_size = 0;
    static char last_path[512] = {0};
    char path[512];
    int dbm = -100;
    
    // Direct string concatenation - much faster than snprintf
    strcpy(path, readcmd);
    strcat(path, "/sta_tp_info");
    
    // Only remap if path changed
    if (strcmp(path, last_path) != 0) {
        if (mapped_data != NULL) {
            munmap(mapped_data, mapped_size);
            mapped_data = NULL;
        }
        
        int fd = open(path, O_RDONLY);
        if (fd < 0) {
            perror("open");
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to open sta_tp_info file for dBm: %s ", path);
#endif
            return -100;  // Return error code
        }
        
        struct stat st;
        if (fstat(fd, &st) < 0) {
            perror("fstat");
            close(fd);
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to stat sta_tp_info file for dBm: %s ", path);
#endif
            return -100;  // Return error code
        }
        
        mapped_size = st.st_size;
        if (mapped_size == 0) {
            close(fd);
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: No WiFi stations connected (sta_tp_info empty for dBm): %s ", path);
#endif
            return -100;  // Return error code
        }
        
        mapped_data = mmap(NULL, mapped_size, PROT_READ, MAP_PRIVATE, fd, 0);
        close(fd);
        
        if (mapped_data == MAP_FAILED) {
            perror("mmap");
            mapped_data = NULL;
#ifdef DEBUG
            GLOBAL_DEBUG_BUILD(true, "DEBUG: Failed to mmap sta_tp_info file for dBm: %s ", path);
#endif
            return -100;  // Return error code
        }
        
        strcpy(last_path, path);
    }
    
    if (mapped_data == NULL) {
        return dbm;
    }
    
    // Use driver-specific dBm format parser
    dbm = dbm_format_parser(mapped_data);
    if (dbm != -100 && dbm != -1) {  // Valid dBm value found
#ifdef DEBUG
        GLOBAL_DEBUG_BUILD(true, "DEBUG: Found dBm via mmap: %d dBm ", dbm);
#endif
    }
    
    return dbm;
}




void mspLQ(int rssi_osd) {
    char command[128];
    snprintf(command, sizeof(command),
             "echo \"VLQ %d &B &F60 &L30\" > /tmp/MSPOSD.msg",
              rssi_osd);
#ifdef DEBUG
    int result = execute_command(command);
    if (result != 0) {
           GLOBAL_DEBUG_BUILD(true, "Warning: MSP OSD command failed with status %d ", result);
    }
#else
    execute_command(command);
#endif
  
}


void set_bitrate_async(int bitrate_mbps) {
    if (!worker_running) return;
    
    pthread_mutex_lock(&worker_mutex);
    pending_cmd.type = CMD_SET_BITRATE;
    pending_cmd.data.bitrate_kbps = bitrate_mbps * BITRATE_MBPS_TO_KBPS;
    pthread_mutex_unlock(&worker_mutex);
    
    sem_post(&worker_sem);
}

// Precompute constants for hot path optimization
void precompute_constants() {
    // Precompute VLQ calculation constants for all possible dbm_Min values
    dbm_range_inv_high = PERCENTAGE_CONVERSION / (double)(DEFAULT_DBM_MAX - DBM_THRESHOLD_HIGH);
    dbm_range_inv_medium = PERCENTAGE_CONVERSION / (double)(DEFAULT_DBM_MAX - DBM_THRESHOLD_MEDIUM);
    dbm_range_inv_low = PERCENTAGE_CONVERSION / (double)(DEFAULT_DBM_MAX - DBM_THRESHOLD_LOW);
    dbm_range_inv_fallback = PERCENTAGE_CONVERSION / (double)(DEFAULT_DBM_MAX - (DEFAULT_DBM_MAX - MIN_DBM_DIFFERENCE));
    
    // Precompute bitrate calculation constants  
    bitrate_scale = bitrate_max / PERCENTAGE_CONVERSION;
    
    // Precompute signal sampling interval
    signal_sampling_interval_ns = NS_PER_SEC / signal_sampling_freq_hz;
    
    // Precompute derived values for configuration display
    signal_sampling_freq_hz_display = (float)fps / signal_sampling_interval;
    emergency_cooldown_frames_display = (float)emergency_cooldown_ms * fps / 1000.0f;
    
    printf("Precomputed constants:\n");
    printf("  dBm range inverse (high): %.6f\n", dbm_range_inv_high);
    printf("  dBm range inverse (medium): %.6f\n", dbm_range_inv_medium);
    printf("  dBm range inverse (low): %.6f\n", dbm_range_inv_low);
    printf("  dBm range inverse (fallback): %.6f\n", dbm_range_inv_fallback);
    printf("  Bitrate scale: %.6f\n", bitrate_scale);
    printf("  Signal sampling interval: %ld ns\n", signal_sampling_interval_ns);
}

int main(int argc, char *argv[]) {
    // Load configuration FIRST - before any config values are used
    config("/etc/ap_alink.conf", &bitrate_max, wificard, &race_mode, &fps,
           &kalman_rssi_process, &kalman_rssi_measure, &kalman_dbm_process, &kalman_dbm_measure,
           &strict_cooldown_ms, &up_cooldown_ms, &min_change_percent,
           &emergency_rssi_threshold, &emergency_bitrate,
           &pid_kp, &pid_ki, &pid_kd,
           &lpf_cutoff_freq, &lpf_sample_freq,
           rssi_filter_chain_config, dbm_filter_chain_config,
           racing_rssi_filter_chain_config, racing_dbm_filter_chain_config,
           racing_video_resolution, &racing_exposure, &racing_fps,
           &signal_sampling_interval, &emergency_cooldown_ms, &control_algorithm,
           &signal_sampling_freq_hz, &hardware_rssi_offset, &cooldown_enabled, &auto_exposure_enabled);
    
    // Only declare truly local variables for the main loop
    int rssi = 0;
    int aDb = 0;
    int currentDb = 0;
    int dbm = DEFAULT_INITIAL_DBM;
    int loop_counter = 0;  // Counter to reduce I/O frequency
    
    // Runtime state variables (not config)
    int bitrate = DEFAULT_BITRATE;
    int bitrate_min = DEFAULT_BITRATE_MIN;
    int histeris = DEFAULT_HYSTERESIS;      // Reduced hysteresis for faster racing response
    int minushisteris = DEFAULT_MINUS_HYSTERESIS; // Reduced hysteresis for faster racing response
    
    // Filtered values (these are runtime state, not config)
    float filtered_rssi = DEFAULT_RSSI_ESTIMATE;
    float filtered_dbm = DEFAULT_DBM_ESTIMATE;
    
    // Precompute constants for hot path optimization
    precompute_constants();
    
    // Initialize driver path AFTER config is loaded
    wifi_card_init(wificard, driverpath);
    
    
    // Initialize filters with config values
    init_filters(kalman_rssi_process, kalman_rssi_measure, kalman_dbm_process, kalman_dbm_measure,
                 lpf_cutoff_freq, lpf_sample_freq);
    
    

    
    // Print signal sampling configuration
    printf("Signal sampling interval: %d frames (%.1fHz at %d FPS)\n", 
           signal_sampling_interval, signal_sampling_freq_hz_display, fps);
    
    // Print emergency cooldown configuration
    printf("Emergency cooldown: %lums (%.1f frames at %d FPS)\n", 
           emergency_cooldown_ms, emergency_cooldown_frames_display, fps);
    
    
    // Set real-time priority for ultra-high performance racing VTX
    set_realtime_priority();
    
    // Initialize signal sampling timing (independent of frame rate)
    struct timespec last_signal_time = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &last_signal_time);
    
    printf("Signal sampling frequency: %d Hz (%.2f ms interval)\n", 
           signal_sampling_freq_hz, signal_sampling_interval_ns / 1000000.0);
    printf("Hardware RSSI offset: %d dBm\n", hardware_rssi_offset);
    printf("Dynamic RSSI thresholds enabled (MCS-based)\n");
    
    // Print cooldown system status
    if (cooldown_enabled) {
        printf("Cooldown system: ENABLED (asymmetric timing)\n");
    } else {
        printf("Cooldown system: DISABLED (immediate bitrate changes)\n");
    }
    
    
    // Initialize pre-calculated sleep values for performance
    init_sleep_values();
    
    // Initialize RSSI reading method function pointer for performance
    init_rssi_read_method();
    
    // Initialize control algorithm function pointer for performance
    init_control_algorithm();
    
    // Initialize PIT mode function pointer for performance
    init_pit_mode();
    
    // Initialize performance lookup tables for optimization
    init_performance_lookup_tables();
    
    
    // Initialize debug output function pointer for performance
    init_debug_output();
    
    // Initialize cooldown check function pointer for performance
    init_cooldown_check();
    
    // Initialize worker thread
    init_worker_thread();
    
    // Initialize counter-based sampling for optimized signal sampling
    init_counter_sampling();
    
    // Initialize race mode configuration
    init_race_mode();
    
    // Initialize HTTP system and video configuration
    init_http_system();
    
    // Enable global debug system after initialization is complete
    enable_global_debug();
    
    // Startup complete - beginning adaptive link control immediately
    printf("Startup complete - beginning adaptive link control\n");
    while (1) {
        // Main control loop - efficient sleep management
        bool work_done_this_iteration = false;
        
        loop_counter++;
        
        // Counter-based signal sampling (not tied to frame rate)
        // This allows for higher frequency signal sampling than frame rate with ZERO CPU overhead
        if (should_sample_signal_counter()) {
            work_done_this_iteration = true; // We're doing signal processing
            new_signal_data_available = true;
            
            // Read signal data
            currentDb = dbm - aDb;
            
            // Try to get dBm from optimized reading method first
            dbm = get_dbm(driverpath);
            
            // If that fails or returns invalid value, try sta_tp_info
            if (dbm == -100 || dbm == 0) {
                int alt_dbm = get_dbm_from_sta_tp_info(driverpath);
                if (alt_dbm != -100 && alt_dbm != -1) {  // Check for error return
                    dbm = alt_dbm;
#ifdef DEBUG
                    GLOBAL_DEBUG_BUILD(true, "DEBUG: Using dBm from sta_tp_info: %d ", dbm);
#endif
                }
            }
            
            aDb = dbm; 
            rssi = get_rssi(driverpath);
            
            // Check for RSSI read error
            if (rssi == -1) {
#ifdef DEBUG
                GLOBAL_DEBUG_BUILD(true, "DEBUG: RSSI read failed, using last known value: %d ", rssi);
#endif
                rssi = 0;  // Use 0 as fallback for error condition
            }
            
            // Apply active filter chains to smooth the signals
            // No more if statement - active filter chains are switched by toggle_racemode()
            filtered_rssi = apply_filter_chain(active_rssi_filter_chain, (float)rssi);
            filtered_dbm = apply_filter_chain(active_dbm_filter_chain, (float)dbm);
            
            // Check for emergency drop conditions
            check_emergency_drop(last_bitrate, filtered_rssi, emergency_rssi_threshold, emergency_bitrate);
        }
        
        // Process signal data if available (hardware clock optimized)
        if (new_signal_data_available) {
            new_signal_data_available = false;
            // Signal data already processed above, just clear the flag
        }
        
        //calculation of dbm_Max dbm_Min
        // dbm_Max is constant, no need to assign every iteration
        int dbm_Min = (filtered_rssi > HIGH_RSSI_THRESHOLD) ? DBM_THRESHOLD_HIGH : (filtered_rssi >= MEDIUM_RSSI_THRESHOLD ? DBM_THRESHOLD_MEDIUM : DBM_THRESHOLD_LOW);

        // Ensure dbm_Min is always less than dbm_Max to prevent divide by zero
        if (dbm_Min >= DEFAULT_DBM_MAX) {
            dbm_Min = DEFAULT_DBM_MAX - MIN_DBM_DIFFERENCE;  // Force at least 1dB difference
        }

        // Calculate VLQ with bounds checking (optimized with precomputed constants)
        double vlq;
        if (DEFAULT_DBM_MAX == dbm_Min) {
            // Fallback: if somehow they're still equal, use RSSI-based calculation
            vlq = (filtered_rssi > 0) ? filtered_rssi : 0.0;  // Optimized: no division needed
        } else {
            // Additional safety check for invalid dBm values
            if (filtered_dbm < -100 || filtered_dbm > 0) {
                // Invalid dBm reading, use RSSI fallback
                vlq = (filtered_rssi > 0) ? filtered_rssi : 0.0;  // Optimized: no division needed
            } else {
                // Optimized VLQ calculation using precomputed range inverses
                double dbm_range_inv_current;
                if (dbm_Min == DBM_THRESHOLD_HIGH) {
                    dbm_range_inv_current = dbm_range_inv_high;
                } else if (dbm_Min == DBM_THRESHOLD_MEDIUM) {
                    dbm_range_inv_current = dbm_range_inv_medium;
                } else if (dbm_Min == DBM_THRESHOLD_LOW) {
                    dbm_range_inv_current = dbm_range_inv_low;
                } else {
                    // Fallback case (DEFAULT_DBM_MAX - MIN_DBM_DIFFERENCE)
                    dbm_range_inv_current = dbm_range_inv_fallback;
                }
                vlq = (filtered_dbm - dbm_Min) * dbm_range_inv_current;
            }
        }

        // Clamp VLQ to valid range (0-100%)
        if (vlq < 0.0) vlq = 0.0;
        if (vlq > VLQ_MAX_THRESHOLD) vlq = VLQ_MAX_THRESHOLD;
      
#ifdef DEBUG
        // Global debug logging - build single string per loop
        if (new_signal_data_available && GLOBAL_DEBUG_THROTTLE(10)) {  // Show every 10th sample
            GLOBAL_DEBUG_RESET();
            GLOBAL_DEBUG_APPEND("Signal: RSSI=%d(%.1f) dBm=%d(%.1f) VLQ=%.2f%% ", 
                        rssi, filtered_rssi, dbm, filtered_dbm, vlq);
            GLOBAL_DEBUG_APPEND("Range: %d-%d Current: %d ", 
                        DEFAULT_DBM_MAX, dbm_Min, currentDb);
        }
#endif
        mspLQ((int)filtered_rssi);
        if (rssi == 0) {
#ifdef DEBUG
GLOBAL_DEBUG_APPEND("ERROR: RSSI is zero - driver path issue detected!\n");
GLOBAL_DEBUG_APPEND("Current driver path: %s\n", driverpath);
GLOBAL_DEBUG_APPEND("Please check WiFi card configuration and driver paths\n");
GLOBAL_DEBUG_APPEND("System will continue but bitrate changes are disabled\n");
GLOBAL_DEBUG_FLUSH(); // Flush debug messages before continue
#endif
            main_loop_sleep(true, false); // Error condition sleep
            continue;
        }

        // Skip bitrate control if WiFi driver is not available
        if (!wifi_driver_available) {
#ifdef DEBUG
 GLOBAL_DEBUG_APPEND("WiFi driver not available - skipping bitrate control\n");
GLOBAL_DEBUG_FLUSH(); // Flush debug messages before continue
#endif
            main_loop_sleep(true, false); // Error condition sleep
            continue;
        }

        // RSSI fallback
            // Clamp VLQ between 0 and 100
            if ( currentDb > histeris || currentDb < minushisteris ) {
                if (vlq > VLQ_MAX_THRESHOLD || rssi > HIGH_RSSI_THRESHOLD) {
                    bitrate = bitrate_max;
                }
            else if (vlq < VLQ_MIN_THRESHOLD || rssi < LOW_RSSI_THRESHOLD) {
                bitrate = bitrate_min;
            }
             else {
                // Calculate target bitrate using VLQ (optimized with precomputed constants)
                int target_bitrate = (int)(vlq * bitrate_scale);
                
                // Use function pointer for control algorithm (no runtime conditionals)
                bitrate = control_algorithm_function(target_bitrate, last_bitrate, &bitrate_pid);
                    
#ifdef DEBUG
                    if (new_signal_data_available) {
                    debug_output_function(target_bitrate, last_bitrate, bitrate);
                    }
#endif
                
                // Clamp bitrate to valid range
                if (bitrate < bitrate_min) bitrate = bitrate_min;
                if (bitrate > bitrate_max) bitrate = bitrate_max;
            }
            }

            
            // Apply cooldown logic and handle bitrate changes (optimized - no runtime conditionals)
            if (cooldown_check_function(bitrate, last_bitrate, strict_cooldown_ms, up_cooldown_ms, min_change_percent, emergency_cooldown_ms)) {
                set_bitrate_async(bitrate);
                set_mcs_async(bitrate, driverpath);
                
#ifdef DEBUG
                GLOBAL_DEBUG_BUILD(true, "Bitrate: %d kbps (RSSI: %.1f, dBm: %.1f) ", 
                       bitrate, filtered_rssi, filtered_dbm);
#endif
            } else {
#ifdef DEBUG
                GLOBAL_DEBUG_BUILD(true, "Cooldown: T=%d C=%d ", bitrate, last_bitrate);
#endif
            }
                
        // Main loop sleep - simple or smart based on config
        main_loop_sleep(false, work_done_this_iteration);
                
#ifdef DEBUG
        // Flush any remaining debug data at the end of each loop iteration
        GLOBAL_DEBUG_FLUSH();
#endif
    }
    
    // Cleanup worker thread
    worker_running = 0;
    sem_post(&worker_sem);  // Wake up worker to exit
    pthread_join(worker_thread, NULL);
    sem_destroy(&worker_sem);

    return 0;
}
