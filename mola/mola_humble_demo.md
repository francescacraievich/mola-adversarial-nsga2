# MOLA SLAM Demo - ROS2 Humble Complete Guide

## TERMINAL 1: Isaac Sim
```bash
cd ~/isaac-sim
./isaac-sim.selector.sh
```
Then: File → Open → carter scene → PLAY

---

## TERMINAL 2: Intensity Node
```bash
cd ~/Fincantieri.Humanoid
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/zIsaac_sim_test/carter_mola_slam/scripts/add_intensity_node.py
```

---

## TERMINAL 3: Verify Topics (after PLAY in Isaac Sim)
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /carter/lidar_with_intensity --once | grep -E "(frame_id|intensity)"
```

---

## TERMINAL 4: MOLA with GUI (Real-Time Mapping)
```bash
cd ~/Fincantieri.Humanoid
source /opt/ros/humble/setup.bash
ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
  lidar_topic_name:=/carter/lidar_with_intensity \
  use_mola_gui:=True
```

---

## TERMINAL 5: Teleop
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel -p use_sim_time:=true
```
**Commands:** `i` forward, `,` backward, `j` left, `l` right, `k` stop

---

## TERMINAL 6: Bag Recording
```bash
cd ~/Fincantieri.Humanoid/mola/data/carter
source /opt/ros/humble/setup.bash
mkdir -p carter_humble_bags
cd carter_humble_bags
ros2 bag record \
  -o carter_demo_humble \
  /carter/lidar_with_intensity \
  /tf \
  /tf_static
```
Record for 60-90 seconds, then `Ctrl+C`

---

## STOP ALL (in order):
1. TERMINAL 6: `Ctrl+C` (stop bag)
2. TERMINAL 5: `Ctrl+C` (stop teleop)
3. TERMINAL 4: `Ctrl+C` (stop MOLA)
4. TERMINAL 2: `Ctrl+C` (stop intensity node)
5. Isaac Sim: STOP button

---

## BAG PROCESSING AND MAP GENERATION

### Verify Bag
```bash
cd ~/Fincantieri.Humanoid/mola/data/carter/carter_humble_bags
ros2 bag info carter_demo_humble
```

### Generate Map from Bag
```bash
cd ~/Fincantieri.Humanoid/mola/data/carter/carter_humble_bags
source /opt/ros/humble/setup.bash

MOLA_LIDAR_TOPIC=/carter/lidar_with_intensity \
mola-lidar-odometry-cli \
  --input-rosbag2 carter_demo_humble \
  --lidar-sensor-label /carter/lidar_with_intensity \
  -c /opt/ros/humble/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
  --output-simplemap carter_humble_output.simplemap
```

### Move Map
```bash
cd ~/Fincantieri.Humanoid/mola/data/carter/carter_humble_bags
mv carter_humble_output.simplemap ../carter_humble_map.simplemap
```

### Convert to Metric Map
```bash
cd ~/Fincantieri.Humanoid/mola/data/carter
source /opt/ros/humble/setup.bash
sm2mm -i carter_humble_map.simplemap -o carter_humble_map.mm
```

---

## VISUALIZATION

### Static Map (mm-viewer)
```bash
cd ~/Fincantieri.Humanoid/mola/data/carter
source /opt/ros/humble/setup.bash
mm-viewer carter_humble_map.mm
```

**mm-viewer Controls:**
- Left mouse + drag: Rotate view
- Scroll wheel: Zoom
- Right mouse: Pan
- `R`: Reset view
- `ESC`: Close

### Review Bag with MolaViz (Dynamic)
```bash
cd ~/Fincantieri.Humanoid/mola/data/carter/carter_humble_bags
source /opt/ros/humble/setup.bash
export MOLA_LIDAR_TOPIC=/carter/lidar_with_intensity
mola-lo-gui-rosbag2 carter_demo_humble
```

---

## GENERATED FILES

After the demo you will have:
```
mola/data/carter/
├── carter_humble_bags/
│   └── carter_demo_humble/
│       ├── carter_demo_humble_0.db3    (ROS2 Humble Bag)
│       └── metadata.yaml                (Humble Metadata)
├── carter_humble_map.simplemap         (SimpleSLAM Map)
└── carter_humble_map.mm                (Metric Map)
```
