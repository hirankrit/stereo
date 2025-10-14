#!/bin/bash
# Script to enable IMX219 Stereo Camera on Jetson Orin Nano

echo "===== IMX219 Stereo Camera Setup for Jetson Orin Nano ====="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
fi

# Backup extlinux.conf
echo "1. Backing up extlinux.conf..."
cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.backup
echo "   Backup saved to: /boot/extlinux/extlinux.conf.backup"
echo ""

# Detect the correct DTB file for Orin Nano Super
DTB_FILE=$(ls /boot/*p3767*-nv-super.dtb 2>/dev/null | head -1)
if [ -z "$DTB_FILE" ]; then
    echo "ERROR: Could not find Orin Nano Super DTB file!"
    exit 1
fi

# Check if dual IMX219 overlay exists
OVERLAY_FILE="/boot/tegra234-p3767-camera-p3768-imx219-dual.dtbo"
if [ ! -f "$OVERLAY_FILE" ]; then
    echo "ERROR: $OVERLAY_FILE not found!"
    echo "Please check your JetPack installation."
    exit 1
fi

echo "2. Found DTB file: $DTB_FILE"
echo "   Found overlay file: $OVERLAY_FILE"
echo ""

# Add FDT overlay to extlinux.conf
echo "3. Adding FDT overlay to boot configuration..."

# Find the line with "APPEND" in the primary label and add FDT before it
sed -i '/LABEL primary/,/^LABEL\|^$/ {
    /APPEND/ {
        i\      FDT '"$DTB_FILE"' '"$OVERLAY_FILE"'
    }
}' /boot/extlinux/extlinux.conf

echo "   FDT overlay added to extlinux.conf"
echo ""

# Verify the changes
echo "4. Verifying changes..."
echo "-----------------------------------"
grep -A 3 "LABEL primary" /boot/extlinux/extlinux.conf | head -5
echo "-----------------------------------"
echo ""

echo "5. Setup complete!"
echo ""
echo "NEXT STEPS:"
echo "  1. Reboot your Jetson Orin Nano: sudo reboot"
echo "  2. After reboot, run: python3 test_stereo_camera.py"
echo ""
echo "To restore original configuration:"
echo "  sudo cp /boot/extlinux/extlinux.conf.backup /boot/extlinux/extlinux.conf"
echo ""
