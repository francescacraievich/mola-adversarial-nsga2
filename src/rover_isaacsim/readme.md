# Unitree G1 Robot Simulation - Isaac Sim
 
A complete robotics simulation featuring the Unitree G1 humanoid robot with sensors in a ship environment, integrated with ROS2 for control and data streaming.
 
## 🤖 Simulation Overview
 
### **Robot:** 

- **Unitree G1** (29 DOF with hands)

- **Sensors:** RTX LiDAR 360° + RGB Camera

- **Articulation Root:** Pelvis link
 
### **Environment:**

- Ship interior with two floor levels

- Collision meshes (work in progress)
 
### **ROS2 Integration:**

- **Publishers:** IMU, Odometry, Joint States, Laser Scan, Point Cloud, RGB Images

- **Subscribers:** Joint Commands, Pose Commands
 
---
 
## 🚀 Quick Start
 
### **1. Launch Isaac Sim:**

```bash

cd /path/to/isaac-sim

./isaac-sim.selector.sh

# ✅ Enable ROS2 Bridge when prompted

```
 
### **2. Load Scene:**

```

File → Open → g1_isaacSim_test.usd

```
 
### **3. Start Simulation:**

- Press **Play** button in Isaac Sim

- ROS2 bridge will activate automatically
 
---
 
## 📡 ROS2 Topics
 
### **Published Topics (Isaac Sim → ROS2):**

```bash

/imu                    # Robot IMU data

/odom                   # Robot odometry

/joint_states           # Current joint positions/velocities/efforts

/scan                   # LiDAR LaserScan

/livox/lidar            # LiDAR PointCloud2

/camera/image           # RGB camera images

/tf                     # Transform tree

/tf_odom                # Transform tree world - odometry

```
 
### **Subscribed Topics (ROS2 → Isaac Sim):**

```bash

/joint_commands          # Control robot joints

/pose_command           # Teleport robot position

```
 
---
 
## 🎮 Robot Control Commands

### **Move Individual Joints (Float64MultiArray ros2 msg):**

### **Raise Left Arm (Shoulder + Elbow)**

```bash
ros2 topic pub /joints_position_cmd std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### **Waist Turn + Right Elbow Bend**

```bash
bashros2 topic pub /joints_position_cmd std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### **Left Leg Kick (Hip + Knee)**

```bash
bashros2 topic pub /joints_position_cmd std_msgs/msg/Float64MultiArray "data: [0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

#### Joint Order Reference (Index : Joint Name):

- index 0  : left_hip_pitch_joint
- index 1  : right_hip_pitch_joint
- index 2  : waist_yaw_joint
- index 3  : left_hip_roll_joint
- index 4  : right_hip_roll_joint
- index 5  : waist_roll_joint
- index 6  : left_hip_yaw_joint
- index 7  : right_hip_yaw_joint
- index 8  : waist_pitch_joint
- index 9  : left_knee_joint
- index 10 : right_knee_joint
- index 11 : left_shoulder_pitch_joint
- index 12 : right_shoulder_pitch_joint
- index 13 : left_ankle_pitch_joint
- index 14 : right_ankle_pitch_joint
- index 15 : left_shoulder_roll_joint
- index 16 : right_shoulder_roll_joint
- index 17 : left_ankle_roll_joint
- index 18 : right_ankle_roll_joint
- index 19 : left_shoulder_yaw_joint
- index 20 : right_shoulder_yaw_joint
- index 21 : left_elbow_joint
- index 22 : right_elbow_joint
- index 23 : left_wrist_roll_joint
- index 24 : right_wrist_roll_joint
- index 25 : left_wrist_pitch_joint
- index 26 : right_wrist_pitch_joint
- index 27 : left_wrist_yaw_joint
- index 28 : right_wrist_yaw_joint
- index 29 : left_hand_index_0_joint
- index 30 : left_hand_middle_0_joint
- index 31 : left_hand_thumb_0_joint
- index 32 : right_hand_index_0_joint
- index 33 : right_hand_middle_0_joint
- index 34 : right_hand_thumb_0_joint
- index 35 : left_hand_index_1_joint
- index 36 : left_hand_middle_1_joint
- index 37 : left_hand_thumb_1_joint
- index 38 : right_hand_index_1_joint
- index 39 : right_hand_middle_1_joint
- index 40 : right_hand_thumb_1_joint
- index 41 : left_hand_thumb_2_joint
- index 42 : right_hand_thumb_2_joint
 

### **Move Individual Joints (JointState ros2 msg):**
 
#### **Move Right Knee:**

```bash

# Example: Bend right knee to 45 degrees (0.785 radians)

ros2 topic pub -1 /joint_commands sensor_msgs/msg/JointState \

"name: ['right_knee_joint']

position: [0.785]

velocity: [0.0]

effort: [0.0]"

```
 
#### **Move Multiple Joints:**

```bash

# Example: Move right leg joints

ros2 topic pub -1 /joint_commands sensor_msgs/msg/JointState \

"name: ['right_hip_pitch_joint', 'right_knee_joint', 'right_ankle_pitch_joint']

position: [-0.5, 1.0, 0.5]

velocity: [0.0, 0.0, 0.0]

effort: [0.0, 0.0, 0.0]"

```
 
#### **Reset to Standing Position:**

```bash

# Reset all joints to zero (standing pose)

ros2 topic pub -1 /joint_commands sensor_msgs/msg/JointState \

"name: ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',

        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',

        'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 

        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint']

position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

```
 
### **Robot Pose Control:**
 
#### **Teleport Robot to New Position:**

```bash

# Move robot to position (2, 1, 0.5) with no rotation

ros2 topic pub -1 /pose_command geometry_msgs/msg/Pose \

"position: {x: 2.0, y: 1.0, z: 0.5}

orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

```
 
#### **Teleport with Rotation:**

```bash

# Move robot and rotate 90° around Z-axis

ros2 topic pub -1 /pose_command geometry_msgs/msg/Pose \

"position: {x: 0.0, y: 0.0, z: 1.0}

orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}"

```
 
#### **Return to Origin:**

```bash

# Reset robot to starting position

ros2 topic pub -1 /pose_command geometry_msgs/msg/Pose \

"position: {x: 0.0, y: 0.0, z: 1.0}

orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

```
 
---
 
## 📊 Sensor Data
 
### **View Camera Feed:**

```bash

 
# Or use RViz2

rviz2

# Add → Image → Topic: /camera/image

```
 
### **View LiDAR Data:**

```bash

# Point Cloud

rviz2

# Add → PointCloud2 → Topic: /livox/lidar

# Set Fixed Frame: world or base_link(pelvis)

```
 
### **Monitor Robot Status:**

```bash

# Joint states

ros2 topic echo /joint_states
 
# IMU data  

ros2 topic echo /imu
 
# Odometry

ros2 topic echo /odom

```
 
---
 
## 🛠️ System Architecture
 
### **Isaac Sim Components:**

- **Unitree G1 Robot** (NVIDIA asset)

- **RTX LiDAR 360°** sensor

- **RGB Camera** sensor  

- **Ship Environment** meshes

- **OmniGraph** (ROS2 bridge)
 
### **OmniGraph Nodes:**

- **Articulation Controller** (pelvis root)

- **ROS2 Publishers** (sensors, robot state)

- **ROS2 Subscribers** (commands)

- **Transform Publishers** (TF tree)
 
---
 
## 📋 Available Joints
 
### **Leg Joints:**

```

left_hip_pitch_joint, left_hip_roll_joint, left_hip_yaw_joint

left_knee_joint, left_ankle_pitch_joint, left_ankle_roll_joint

right_hip_pitch_joint, right_hip_roll_joint, right_hip_yaw_joint  

right_knee_joint, right_ankle_pitch_joint, right_ankle_roll_joint

```
 
### **Arm Joints:**

```

left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint

left_elbow_joint, left_wrist_pitch_joint, left_wrist_roll_joint

right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint

right_elbow_joint, right_wrist_pitch_joint, right_wrist_roll_joint

```
 
### **Hand Joints:**

```

[Additional hand joints (14) + 29 base DOF in total]

```
 
---
 
## 🔧 Troubleshooting
 
### **Robot Not Responding:**

1. ✅ **Simulation Playing?** Press Play in Isaac Sim

2. ✅ **ROS2 Bridge Active?** Check `ros2 topic list`

3. ✅ **Joint Names Correct?** Check `ros2 topic echo /joint_states`
 
### **Sensors Not Publishing:**

1. ✅ **Topics Available?** `ros2 topic list | grep camera`

2. ✅ **Data Flowing?** `ros2 topic hz /camera/image`
 
### **Robot and objects don't Collide Well:**

1. ✅ **Collision Meshes** need refinement

2. ✅ **Physics Properties** may need adjustment
 
---
 
## 📁 File Structure
 
```

g1_isaacSim_test.usd          # Main simulation scene

├── Unitree G1 Robot          # 29 DOF humanoid + hands

├── RTX LiDAR Sensor          # 360° 3D scanning

├── RGB Camera                # Color + depth imaging  

├── Ship Environment          # Two-floor structure

└── OmniGraph Bridge          # ROS2 integration

```
 
---
 
## 🎯 Next Steps
 
1. **Improve collision meshes** for ship environment

2. **Check sensors data** (frameID, data values, ...)

...
 
---
 
## 📖 Dependencies
 
- **Isaac Sim** (with ROS2 bridge enabled)

- **ROS2** (Humble/Jazzy recommended)

- **Standard ROS2 packages**: `sensor_msgs`, `geometry_msgs`, `tf2_ros`
 
---
 
