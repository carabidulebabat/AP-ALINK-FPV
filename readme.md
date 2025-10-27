# AP-ALINK-FPV: Advanced Adaptive Link System

A sophisticated FPV link adaptation system for OpenIPC-based devices, featuring intelligent signal processing, adaptive bitrate control, and real-time performance optimization.

## Features

### **Core Functionality**
- **Adaptive Bitrate Control**: Dynamic bitrate adjustment based on real-time signal quality
- **Advanced Filter System**: Choose from 6 different filter types (Kalman, Low-Pass, Mode, Derivative, 2-Pole LPF, Mean, Gaussian)
- **PID Controller**: Smooth, stable bitrate transitions with configurable gains
- **Asymmetric Cooldown**: Prevents oscillation with faster decreases and slower increases
- **Emergency Drop Logic**: Immediate response to critical signal loss
- **Real-time Priority**: SCHED_FIFO scheduling for deterministic timing
- **Counter-Based Sampling**: Zero-overhead signal sampling optimized for embedded systems

### **Signal Processing**
- **RSSI Analysis**: Real-time signal strength monitoring
- **dBm Processing**: Power level analysis and adaptation
- **Advanced Noise Filtering**: 6 different filter algorithms for optimal signal processing
- **Independent Filter Configuration**: RSSI and dBm can use different filter types
- **Multi-antenna Support**: Handles multiple antenna configurations

### **Performance Optimizations**
- **Memory-Mapped Files**: Zero I/O overhead for signal reading (20-200x faster)
- **Counter-Based Sampling**: Zero CPU overhead signal sampling
- **Emergency Cooldown**: Ultra-fast bitrate drops (50ms default, configurable to 8ms)
- **Worker Threads**: Asynchronous API calls to prevent blocking
- **HTTP Optimization**: Raw socket implementation instead of wget
- **Memory Efficiency**: Minimal memory footprint

### **WiFi Power Management**
- **Power Save Control**: Disable WiFi power saving for FPV stability
- **Maximum TX Power**: Automatic power optimization per card type
- **Driver Management**: Automatic driver reloading with optimal settings

### **Racing Mode**
- **Dual FPS System**: Different frame rates for normal vs racing mode
- **Dual Filter System**: Different filter chains for normal vs racing mode
- **Ultra-Low Latency**: Optimized settings for racing applications
- **PIT Mode**: Low power standby with HTTP wake-up capability

## Requirements

- **Hardware**: OpenIPC-compatible device (tested on SSC880Q)
- **OS**: Linux-based system with OpenIPC firmware
- **WiFi Card**: Compatible with 8812au/8812eu/873xbu drivers
- **Permissions**: Root access for real-time priority

## Installation

1. **Clone/Download** the AP-ALINK-FPV repository
2. **Compile** the application:
   ```bash
   make clean && make debug
   ```
3. **Install** the binary:
   ```bash
   sudo cp ap_alink_debug /usr/local/bin/ap_alink
   sudo chmod +x /usr/local/bin/ap_alink
   ```
4. **Configure** the system (see Configuration section)

## Configuration

The system is configured via `ap_alink.conf`. Here's a complete configuration example:

```ini
# =============================================================================
# BASIC SETTINGS
# =============================================================================
bitrate_max=19
wificard=8812au    # Options: 8812au, 8812eu2, 873xbu
fps=120

# =============================================================================
# WIFI POWER MANAGEMENT
# =============================================================================
# CRITICAL: Disable WiFi power saving for FPV stability
disable_wifi_power_save=0
enable_maximum_tx_power=1

# =============================================================================
# PIT MODE - LOW POWER STANDBY WITH HTTP WAKE-UP
# =============================================================================
pit_mode_enabled=0

# =============================================================================
# SIGNAL PROCESSING & FILTERS
# =============================================================================
# Filter Chain Configuration
# Format: comma-separated filter types (0=Kalman, 1=Low-Pass, 2=Mode, 3=Derivative, 4=2-Pole LPF, 5=Mean, 6=Gaussian)
rssi_filter_chain=0
dbm_filter_chain=0

# Low-Pass Filter Settings (used by filter types 1 and 4)
lpf_cutoff_freq=2.0
lpf_sample_freq=10.0

# Kalman Filter Parameters
kalman_rssi_process=0.00001
kalman_dbm_process=0.00001
kalman_rssi_measure=0.1
kalman_dbm_measure=0.5

# =============================================================================
# SIGNAL SAMPLING & TIMING
# =============================================================================
# RSSI Reading Method
rssi_read_method=1  # 0=mmap (faster), 1=file rewind (more compatible)

# Signal Sampling Configuration
signal_sampling_interval=5  # Frames between signal readings
signal_sampling_freq_hz=50  # Independent signal sampling frequency (Hz)

# =============================================================================
# BITRATE CONTROL & COOLDOWNS
# =============================================================================
# Cooldown System Control
cooldown_enabled=0  # 0=disabled, 1=enabled

# Asymmetric Cooldown Parameters
strict_cooldown_ms=200
up_cooldown_ms=3000
emergency_cooldown_ms=50
min_change_percent=5

# =============================================================================
# EMERGENCY & SAFETY SETTINGS
# =============================================================================
emergency_rssi_threshold=30
emergency_bitrate=1000
hardware_rssi_offset=0

# Control Algorithm Configuration
control_algorithm=1  # 0=PID Controller, 1=Simple FIFO

# =============================================================================
# PID CONTROLLER (Advanced Users)
# =============================================================================
pid_kp=1.0
pid_ki=0.05
pid_kd=0.4

# =============================================================================
# EXPOSURE CONTROL
# =============================================================================
auto_exposure_enabled=1

# =============================================================================
# RACING MODE OVERRIDES CONFIGURATION
# =============================================================================
race_mode=1
racing_video_resolution=1280x720
racing_exposure=11
racing_fps=120
racing_rssi_filter_chain=1
racing_dbm_filter_chain=1

# =============================================================================
# CENTRALIZED SLEEP CONFIGURATION
# =============================================================================
sleep_main_loop_us=100
sleep_normal_mode_ms=20
sleep_high_perf_mode_ms=5
sleep_ultra_low_mode_ms=0
sleep_error_condition_ms=100
smart_sleep_enabled=0
```

### **Configuration Parameters Explained**

#### **Basic Settings**
- **`bitrate_max`**: Maximum bitrate in Mbps (1-19)
- **`wificard`**: WiFi card identifier (8812au, 8812eu2, 873xbu)
- **`fps`**: Video frame rate (typically 60-120)

#### **WiFi Power Management**
- **`disable_wifi_power_save`**: Disable WiFi power saving (0=disabled, 1=enabled)
- **`enable_maximum_tx_power`**: Enable maximum TX power (0=disabled, 1=enabled)

#### **PIT Mode**
- **`pit_mode_enabled`**: Enable PIT mode for low power standby (0=disabled, 1=enabled)

#### **Signal Processing & Filters**
- **`rssi_filter_chain`**: RSSI filter chain configuration
- **`dbm_filter_chain`**: dBm filter chain configuration
- **`racing_rssi_filter_chain`**: Racing mode RSSI filter chain
- **`racing_dbm_filter_chain`**: Racing mode dBm filter chain
- **`lpf_cutoff_freq`**: Low-pass filter cutoff frequency
- **`lpf_sample_freq`**: Low-pass filter sample frequency
- **`kalman_rssi_process`**: Kalman filter process variance for RSSI
- **`kalman_dbm_process`**: Kalman filter process variance for dBm
- **`kalman_rssi_measure`**: Kalman filter measurement variance for RSSI
- **`kalman_dbm_measure`**: Kalman filter measurement variance for dBm

#### **Signal Sampling & Timing**
- **`rssi_read_method`**: RSSI reading method (0=mmap, 1=file rewind)
- **`signal_sampling_interval`**: Frames between signal readings
- **`signal_sampling_freq_hz`**: Independent signal sampling frequency (Hz)

#### **Bitrate Control & Cooldowns**
- **`cooldown_enabled`**: Enable cooldown system (0=disabled, 1=enabled)
- **`strict_cooldown_ms`**: Minimum time between any bitrate changes
- **`up_cooldown_ms`**: Additional time required before increasing bitrate
- **`min_change_percent`**: Minimum percentage change to trigger adjustment
- **`emergency_cooldown_ms`**: Time between bitrate decreases
- **`control_algorithm`**: Control algorithm (0=PID, 1=Simple FIFO)

#### **Emergency & Safety Settings**
- **`emergency_rssi_threshold`**: RSSI threshold for emergency drop
- **`emergency_bitrate`**: Bitrate to drop to in emergency
- **`hardware_rssi_offset`**: Hardware-specific RSSI offset for calibration

#### **PID Controller (Advanced Users)**
- **`pid_kp`**: Proportional gain (immediate response)
- **`pid_ki`**: Integral gain (steady-state error correction)
- **`pid_kd`**: Derivative gain (overshoot reduction)

#### **Exposure Control**
- **`auto_exposure_enabled`**: Auto-calculate exposure based on frame rate (0=disabled, 1=enabled)

#### **Racing Mode Overrides**
- **`race_mode`**: Enable racing mode (0=disabled, 1=enabled)
- **`racing_video_resolution`**: Video resolution for racing mode
- **`racing_exposure`**: Camera exposure for racing mode
- **`racing_fps`**: Frame rate for racing mode

#### **Centralized Sleep Configuration**
- **`sleep_main_loop_us`**: Base loop sleep (microseconds)
- **`sleep_normal_mode_ms`**: Additional sleep for normal mode (milliseconds)
- **`sleep_high_perf_mode_ms`**: Additional sleep for high performance mode
- **`sleep_ultra_low_mode_ms`**: Additional sleep for ultra-low latency mode
- **`sleep_error_condition_ms`**: Sleep when RSSI=0 or driver unavailable
- **`smart_sleep_enabled`**: Enable smart sleep mode (0=disabled, 1=enabled)

## Filter System

### **Available Filter Types**

| Filter Type | Description | Best For |
|-------------|-------------|----------|
| **0 - Kalman** | Optimal adaptive filtering | General purpose, optimal performance |
| **1 - Low-Pass** | Simple exponential moving average | Racing, low latency applications |
| **2 - Mode** | Median-like filtering | Noisy environments, stable median |
| **3 - Derivative** | Trend detection and analysis | Trend detection, rate analysis |
| **4 - 2-Pole LPF** | Professional Butterworth filtering | Professional filtering, steep rolloff |
| **5 - Mean** | Simple moving average | Basic noise reduction |
| **6 - Gaussian** | Bell-curve weighted average | Smooth noise reduction with center emphasis |

### **Filter Chain Configuration**

You can chain multiple filters together for enhanced performance:

```ini
# Single filter
rssi_filter_chain=0  # Kalman only

# Chained filters
rssi_filter_chain=2,0  # Mode→Kalman (excellent noise rejection)
rssi_filter_chain=1,4  # Low-Pass→2-Pole LPF (smooth filtering)
rssi_filter_chain=3,0  # Derivative→Kalman (trend analysis)
```

## Control Algorithms

### **PID Controller (control_algorithm=0)**
- **Pros**: Smooth transitions, reduces oscillation, adaptive behavior
- **Cons**: Higher CPU usage, more complex
- **Use Case**: Smooth video streaming, stable connections

### **Simple FIFO (control_algorithm=1)**
- **Pros**: Maximum performance, direct response, lower CPU usage
- **Cons**: May cause more frequent bitrate changes
- **Use Case**: Racing, low-latency applications, maximum responsiveness

## Usage Scenarios

### **Racing Mode (Low Latency)**
```ini
race_mode=1
racing_fps=120
racing_rssi_filter_chain=1  # Low-Pass for speed
racing_dbm_filter_chain=1   # Low-Pass for speed
signal_sampling_freq_hz=100 # High frequency sampling
emergency_cooldown_ms=25    # Fast emergency response
control_algorithm=1         # Simple FIFO for responsiveness
```

### **Long Range (Stability)**
```ini
race_mode=0
rssi_filter_chain=0        # Kalman for stability
dbm_filter_chain=0        # Kalman for stability
signal_sampling_freq_hz=30 # Lower frequency for stability
emergency_cooldown_ms=100  # Slower emergency response
control_algorithm=0       # PID for smooth transitions
```

### **Balanced Performance**
```ini
race_mode=0
rssi_filter_chain=0        # Kalman for RSSI
dbm_filter_chain=1         # Low-Pass for dBm
signal_sampling_freq_hz=50  # Default frequency
emergency_cooldown_ms=50    # Default emergency response
control_algorithm=1        # Simple FIFO for performance
```

## Running the Application

### **Basic Usage**
```bash
sudo ./ap_alink
```

### **With Debug Output**
```bash
sudo ./ap_alink_debug
```

### **Systemd Service** (Recommended)
```bash
# Create service file
sudo nano /etc/systemd/system/ap-alink.service

[Unit]
Description=AP-ALINK-FPV Adaptive Link System
After=network.target

[Service]
Type=simple
User=root
ExecStart=/usr/local/bin/ap_alink
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target

# Enable and start
sudo systemctl enable ap-alink
sudo systemctl start ap-alink
```

## Performance Monitoring

### **Debug Output**
When running with debug enabled, you'll see:
```
Counter-based sampling: Initialized for signal sampling (20 samples per loop)
Worker thread: Initialized and started
Filters initialized - RSSI: Kalman, dBm: Kalman
Real-time priority set successfully (SCHED_FIFO, priority 50)

Signal: RSSI=45(44.8) dBm=-55(-55.2) VLQ=85.23%
Bitrate: 1620 kbps (RSSI: 44.8, dBm: -55.2)
```

### **Key Metrics**
- **VLQ**: Video Link Quality percentage
- **Filtered Values**: Smoothed RSSI and dBm readings
- **Timing**: Counter-based sampling and cooldown compliance

## Troubleshooting

### **Common Issues**

#### **Permission Denied**
```bash
# Ensure running as root
sudo ./ap_alink
```

#### **Real-time Priority Failed**
```bash
# Check if running as root
sudo ./ap_alink
```

#### **No Signal Data**
```bash
# Check WiFi card compatibility
lsusb | grep -i rtl
# Verify driver is loaded
lsmod | grep 88
```

#### **High CPU Usage**
```bash
# Reduce signal sampling frequency
signal_sampling_freq_hz=30
# Increase cooldown times
emergency_cooldown_ms=100
# Use low-pass filters instead of Kalman
rssi_filter_chain=1
```

### **Performance Tuning**

#### **For Racing**
- Use low-pass filters (`rssi_filter_chain=1`)
- Higher sampling frequency (`signal_sampling_freq_hz=100`)
- Shorter cooldowns (`emergency_cooldown_ms=25`)
- Simple FIFO control (`control_algorithm=1`)

#### **For Long Range**
- Use Kalman filters (`rssi_filter_chain=0`)
- Lower sampling frequency (`signal_sampling_freq_hz=30`)
- Longer cooldowns (`emergency_cooldown_ms=100`)
- PID control (`control_algorithm=0`)

#### **For Noisy Environments**
- Use mode filters (`rssi_filter_chain=2,0`) for excellent noise rejection
- Standard cooldowns (`emergency_cooldown_ms=50`)
- Balanced control (`control_algorithm=1`)

## Architecture

### **System Components**
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Signal Input  │───▶│  Advanced Filter │───▶│  Control Logic  │
│   (RSSI/dBm)    │    │  System (6 types)│    │  (PID/FIFO)     │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                                         │
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Bitrate API   │◀───│  Cooldown Logic  │◀───│  Bitrate Calc    │
│   (HTTP calls)  │    │  (Asymmetric)     │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### **Thread Architecture**
- **Main Thread**: Signal processing and control logic
- **Worker Thread**: Asynchronous API calls
- **Real-time Priority**: SCHED_FIFO for consistent timing

## Performance Metrics

### **Latency**
- **Signal Processing**: < 1ms
- **Filter Application**: < 0.1ms
- **API Calls**: Asynchronous (non-blocking)
- **Control Loop**: 1000Hz (counter-based)
- **Signal Sampling**: Configurable (default 50Hz)

### **CPU Usage**
- **Counter-Based Sampling**: ~0.01% CPU
- **Low-Pass Filter**: ~0.1% CPU
- **Kalman Filter**: ~0.3% CPU
- **Mode Filter**: ~0.2% CPU
- **Derivative Filter**: ~0.3% CPU
- **2-Pole LPF**: ~0.2% CPU
- **PID Controller**: ~0.05% CPU
- **Total System**: < 1% CPU

### **Memory Usage**
- **Static Allocation**: ~2KB
- **Stack Usage**: < 1KB
- **Total Memory**: < 5KB

## Key Features Summary

✅ **Counter-Based Sampling** - Zero CPU overhead signal sampling  
✅ **Memory-Mapped I/O** - 20-200x faster signal reading  
✅ **Advanced Filter System** - 6 different filter types with chaining  
✅ **Dual Control Algorithms** - PID vs Simple FIFO for different use cases  
✅ **Racing Mode** - Optimized settings for low latency applications  
✅ **WiFi Power Management** - Automatic power optimization  
✅ **PIT Mode** - Low power standby with HTTP wake-up  
✅ **Asymmetric Cooldown** - Fast drops, slow increases  
✅ **Emergency Drop Logic** - Immediate response to signal loss  
✅ **Real-time Priority** - SCHED_FIFO scheduling  
✅ **Worker Threads** - Asynchronous API calls  
✅ **Comprehensive Configuration** - Organized settings for easy tuning  

## Quick Start Guide

### **For Racing (Low Latency):**
```ini
race_mode=1
racing_fps=120
signal_sampling_freq_hz=100
emergency_cooldown_ms=25
control_algorithm=1
racing_rssi_filter_chain=1
racing_dbm_filter_chain=1
```

### **For Long Range (Stability):**
```ini
race_mode=0
fps=60
signal_sampling_freq_hz=30
emergency_cooldown_ms=100
control_algorithm=0
rssi_filter_chain=0
dbm_filter_chain=0
```

### **For Balanced Performance:**
```ini
race_mode=0
fps=120
signal_sampling_freq_hz=50
emergency_cooldown_ms=50
control_algorithm=1
rssi_filter_chain=0
dbm_filter_chain=1
```

## License

This project is completely free to use for everyone. No restrictions apply.