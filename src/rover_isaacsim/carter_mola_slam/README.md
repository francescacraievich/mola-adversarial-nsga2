# MOLA SLAM per Nova Carter - Isaac Sim

Configurazione MOLA per mappatura 3D con il LiDAR Pandar XT-32 del Nova Carter.

## Avvio Rapido

### 1. Avvia Isaac Sim con carter_isaac_sim.usd
### 2. Verifica LiDAR: ros2 topic hz /carter/lidar
### 3. Lancia MOLA:

```bash
cd ~/Fincantieri.Humanoid/src/zIsaac_sim_test/carter_mola_slam
ros2 launch launch/carter_mola_slam.launch.py
```

### 4. Visualizza in RViz2:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

Configura Fixed Frame: odom
Aggiungi: PointCloud2 (/carter/lidar), PointCloud2 (/mola/map), Path (/mola/trajectory)

## File Creati

- config/carter_mola_lidar_3d.yaml - Configurazione MOLA
- launch/carter_mola_slam.launch.py - Launch file

Vedi commenti nei file per dettagli configurazione.
