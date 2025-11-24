#!/bin/bash
# Script to install MOLA and dependencies for ROS2 Jazzy
# For mola-adversarial-nsga2 project

set -e

echo "========================================="
echo "Installing MOLA for ROS2 Jazzy"
echo "========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Verify ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 not found. Please install ROS2 Jazzy first."
    exit 1
fi

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

if [ "$ROS_DISTRO" != "jazzy" ]; then
    print_warning "This installation is for Jazzy. You are using: $ROS_DISTRO"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

print_status "ROS Distribution: $ROS_DISTRO"
echo ""

# Update package lists
print_status "Updating package lists..."
sudo apt update

echo ""
print_status "Installing MRPT packages (base dependencies)..."
sudo apt install -y \
    ros-jazzy-mrpt-libbase \
    ros-jazzy-mrpt-libmath \
    ros-jazzy-mrpt-libposes \
    ros-jazzy-mrpt-libmaps \
    ros-jazzy-mrpt-libobs \
    ros-jazzy-mrpt-libgui \
    ros-jazzy-mrpt-libslam || print_warning "Some MRPT packages not installed"

echo ""
print_status "Installing MOLA core packages..."
sudo apt install -y \
    ros-jazzy-mola-kernel \
    ros-jazzy-mola-common \
    ros-jazzy-mola-launcher || print_error "MOLA core installation failed"

echo ""
print_status "Installing MOLA SLAM components..."
sudo apt install -y \
    ros-jazzy-mola-lidar-odometry \
    ros-jazzy-mola-metric-maps \
    ros-jazzy-mola-input-rosbag2 \
    ros-jazzy-mola-bridge-ros2 || print_warning "Some SLAM components not installed"

echo ""
print_status "Installing MOLA tools and utilities..."
sudo apt install -y \
    ros-jazzy-mola-demos \
    ros-jazzy-mola-viz || print_warning "Some demo packages not installed"

echo ""
print_status "Installing additional ROS2 packages..."
sudo apt install -y \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins || print_warning "Some ROS2 packages not installed"

echo ""
print_status "Installing Python dependencies..."
pip3 install --break-system-packages numpy opencv-python scipy PyYAML psutil 2>/dev/null || \
    pip3 install numpy opencv-python scipy PyYAML psutil

# Check for GPU
if lspci | grep -i nvidia &> /dev/null; then
    print_status "NVIDIA GPU detected, installing GPUtil..."
    pip3 install --break-system-packages GPUtil 2>/dev/null || pip3 install GPUtil
fi

echo ""
echo "========================================="
print_status "✅ Installation completed!"
echo "========================================="
echo ""
echo "📋 Verify installation:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 pkg list | grep mola"
echo ""
echo "📋 Next steps to test MOLA:"
echo "   1. Start Isaac Sim with Carter scene"
echo "   2. Launch intensity node: python3 src/rover_isaacsim/carter_mola_slam/scripts/add_intensity_node.py"
echo "   3. Launch MOLA live mapping (see updated guide)"
echo ""
echo "📖 Check documentation in mola/ directory"
echo ""
