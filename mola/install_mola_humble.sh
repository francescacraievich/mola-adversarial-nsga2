#!/bin/bash
# Script per installare MOLA e dipendenze per ROS2 Humble
# Adattato per Fincantieri Humanoid project

set -e

echo "========================================="
echo "Installazione MOLA per ROS2 Humble"
echo "========================================="
echo ""

# Colori per output
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

# Verifica ROS2
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 non trovato. Installa ROS2 Humble prima."
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash 2>/dev/null || true

if [ "$ROS_DISTRO" != "humble" ]; then
    print_warning "Questa installazione è per Humble. Stai usando: $ROS_DISTRO"
    read -p "Continuare comunque? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

print_status "Distribuzione ROS: $ROS_DISTRO"
echo ""

# Update package lists
print_status "Aggiornamento liste pacchetti..."
sudo apt update

echo ""
print_status "Installazione pacchetti MRPT (dipendenze base)..."
sudo apt install -y \
    ros-humble-mrpt-libbase \
    ros-humble-mrpt-libmath \
    ros-humble-mrpt-libposes \
    ros-humble-mrpt-libmaps \
    ros-humble-mrpt-libobs \
    ros-humble-mrpt-libgui \
    ros-humble-mrpt-libslam \
    ros-humble-mrpt-apps || print_warning "Alcuni pacchetti MRPT non installati"

echo ""
print_status "Installazione pacchetti MOLA core..."
sudo apt install -y \
    ros-humble-mola-kernel \
    ros-humble-mola-common \
    ros-humble-mola-launcher || print_error "Installazione MOLA core fallita"

echo ""
print_status "Installazione MOLA SLAM components..."
sudo apt install -y \
    ros-humble-mola-lidar-odometry \
    ros-humble-mola-metric-maps \
    ros-humble-mola-input-rosbag2 \
    ros-humble-mola-bridge-ros2 || print_warning "Alcuni componenti SLAM non installati"

echo ""
print_status "Installazione MOLA tools e utilities..."
sudo apt install -y \
    ros-humble-mola-demos || print_warning "Demo packages non installati"

echo ""
print_status "Installazione pacchetti ROS2 aggiuntivi..."
sudo apt install -y \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-diagnostic-aggregator \
    ros-humble-tf2-tools \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins || print_warning "Alcuni pacchetti ROS2 non installati"

echo ""
print_status "Installazione dipendenze Python..."
pip3 install --break-system-packages numpy opencv-python scipy PyYAML psutil 2>/dev/null || \
    pip3 install numpy opencv-python scipy PyYAML psutil

# Check GPU
if lspci | grep -i nvidia &> /dev/null; then
    print_status "NVIDIA GPU rilevata, installazione GPUtil..."
    pip3 install --break-system-packages GPUtil 2>/dev/null || pip3 install GPUtil
fi

echo ""
echo "========================================="
print_status "✅ Installazione completata!"
echo "========================================="
echo ""
echo "📋 Verifica installazione:"
echo "   source /opt/ros/humble/setup.bash"
echo "   ros2 pkg list | grep mola"
echo ""
echo "📋 Prossimi passi per testare MOLA:"
echo "   1. Avvia Isaac Sim con la scena Carter"
echo "   2. Lancia il nodo intensity: python3 src/zIsaac_sim_test/carter_mola_slam/scripts/add_intensity_node.py"
echo "   3. Lancia MOLA live mapping (vedi guida aggiornata)"
echo ""
echo "📖 Consulta la documentazione in mola/README.md"
echo ""
