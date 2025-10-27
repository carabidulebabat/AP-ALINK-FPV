#!/bin/bash

# Check if password argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <password>"
    echo "Example: $0 12345"
    exit 1
fi

PASSWORD="$1"
DEVICE_IP="192.168.1.15"
DEVICE_USER="root"

echo "=== AP-ALINK-FPV Simple Deploy Script ==="
echo "Device: $DEVICE_USER@$DEVICE_IP"
echo

# Clean first
echo "Cleaning previous builds..."
make clean

# Build selection
echo "Select build type:"
echo "1) Release build (make)"
echo "2) Debug build (make debug)"
read -p "Enter choice (1 or 2): " BUILD_CHOICE

case $BUILD_CHOICE in
    1)
        echo "Building release version..."
        make
        ;;
    2)
        echo "Building debug version..."
        make debug
        echo "Renaming debug binary to ap_alink..."
        cp ap_alink_debug ap_alink
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

echo "Build complete!"
echo

# Stop existing process
echo "Stopping existing ap_alink process..."
sshpass -p "$PASSWORD" ssh "$DEVICE_USER@$DEVICE_IP" "killall ap_alink 2>/dev/null || true"

# Remount /proc as read-write (required for WiFi driver writes)
echo "Remounting /proc as read-write..."
sshpass -p "$PASSWORD" ssh "$DEVICE_USER@$DEVICE_IP" "mount -o remount,rw /proc"

# Deploy binary
echo "Deploying ap_alink to device..."
sshpass -p "$PASSWORD" scp -O ap_alink "$DEVICE_USER@$DEVICE_IP:/usr/bin/"

# Deploy config file
echo "Deploying config file to device..."
sshpass -p "$PASSWORD" scp -O ap_alink.conf "$DEVICE_USER@$DEVICE_IP:/etc/"

# Ask about restart
echo
echo "Deployment complete!"
echo "Options:"
echo "1) Restart device (reboot)"
echo "2) Just restart ap_alink process"
echo "3) Do nothing (manual restart)"
read -p "Enter choice (1, 2, or 3): " RESTART_CHOICE

case $RESTART_CHOICE in
    1)
        echo "Restarting device..."
        sshpass -p "$PASSWORD" ssh "$DEVICE_USER@$DEVICE_IP" "reboot"
        echo "Device is restarting..."
        ;;
    2)
        echo "Starting ap_alink..."
        sshpass -p "$PASSWORD" ssh "$DEVICE_USER@$DEVICE_IP" "nohup /usr/bin/ap_alink > /tmp/ap_alink_debug.log 2>&1 &"
        echo "ap_alink started in background"
        ;;
    3)
        echo "Manual restart required"
        ;;
    *)
        echo "Invalid choice. Manual restart required."
        ;;
esac

echo "Deploy script complete!"