# Robotic Chair Navigation System

A comprehensive ROS 2-based autonomous navigation system for a robotic wheelchair equipped with advanced sensors and Nav2 integration. The system enables autonomous navigation, localization, and motion planning in hospital environments.

## 🤖 Project Overview

**Robotic Chair** is a simulation-ready robotic wheelchair platform built with ROS 2. It integrates:
- **Nav2 Stack** for autonomous navigation and motion planning
- **AMCL** for robot localization and mapping
- **DWB Local Planner** for collision-free path following
- **Gazebo Simulation** with realistic physics and sensor simulation
- **URDF Model** with complete sensor suite

Perfect for research, education, and autonomous mobile manipulation studies.

## ✨ Key Features

### Navigation & Localization
- ✅ Autonomous navigation with Nav2 stack
- ✅ Adaptive Monte Carlo Localization (AMCL)
- ✅ Dynamic Window Approach (DWA) for local planning
- ✅ Global path planning with Navfn planner
- ✅ Real-time costmap management (local & global)
- ✅ Behavior recovery (spin, backup, assisted teleop)

### Sensors & Perception
- ✅ 2D LIDAR (1000 samples, 12m range, 5 Hz update)
- ✅ Monocular camera (640×480, 15 Hz)
- ✅ 9-DOF IMU (100 Hz)
- ✅ Cliff detection (front)
- ✅ Odometry from differential drive

### Simulation & Control
- ✅ ODE ("Open Dynamics Engine") physics engine with accurate wheel dynamics
- ✅ Gazebo ROS integration (plugins for all sensors)
- ✅ Differential drive controller
- ✅ Multi-room hospital environment
- ✅ Configurable parameters for tuning

## 🏗️ Hardware Architecture

### Drive System
| Component | Specification |
|-----------|--------------|
| **Wheels** | 2 × continuous joints, 0.32m diameter, 0.54m separation |
| **Max Speed** | 0.26 m/s (linear) |
| **Max Rotation** | 1.0 rad/s |
| **Casters** | Front: 2 spherical casters (0.1m radius) |

### Sensors
| Sensor | Details |
|--------|---------|
| **LIDAR** | 2D ray sensor, 1000 samples, 0.4–12m range, -π to π coverage |
| **Camera** | RGB, 640×480px, 80.4° FOV, Gaussian noise |
| **IMU** | 6-axis accel + gyro, 100 Hz, mounted on seat |
| **Cliff Sensor** | Single-beam ray sensor, 0.15m range (front) |

### Chassis Specifications
| Dimension | Value |
|-----------|-------|
| **Length** | 0.7 m |
| **Width** | 0.55 m |
| **Height** | ~0.65 m (with backrest & headrest) |
| **Base Mass** | 30 kg |
| **Seat Mass** | 5 kg |
| **Total (approx)** | 41 kg |

## 📋 Project Structure

```
robotic_chair_description/
├── urdf/
    └── chair.urdf                    # Complete URDF with sensors & physics

robotic_chair_nav/
├── config/
│   ├── nav2_params.yaml              # Core Nav2 parameters
│   └── amcl_params.yaml              # Pre-built hospital map
├── maps/
│   └── hospital_map.yaml             # Navigation map for AMCL
└── launch/
│     └── keyboard_control.launch.py        # Launch file 
└── robotic_chair_nav
      └── navigation_commander.py 

simple_world/
├── worlds/
│   └── robotic_chair_worlds.sdf   # Gazebo world file
```

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 
- ROS 2 (Humble)
- Gazebo (11+)
- Python 3.8+

### Installation

1. **Create ROS 2 workspace**
   ```bash
   mkdir -p ~/robotic_chair_ws/src
   cd ~/robotic_chair_ws
   ```

2. **Clone repository**
   ```bash
   cd src
   git clone https://github.com/IALR/ROS_FULL_NAV.git robotic_chair
   ```

3. **Install dependencies**
   ```bash
   cd ~/robotic_chair_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build workspace**
   ```bash
   colcon build --symlink-install
   ```

5. **Source setup**
   ```bash
   source install/setup.bash
   ```

### Running the Simulation

**Luanching process: Full stack (Gazebo + Nav2 + RViz)**
```bash
ros2 launch robotic_chair_nav keyboard_control.launch.py     # Nav2 stack launch

```

This will:
- Launch Gazebo with the 4-room hospital world
- Spawn the robotic chair at origin (0, 0)
- Start Nav2 stack with AMCL localization
- Open RViz2 with default Nav2 configuration

## 🎮 Using the Navigation System

### 1. Initial Localization

In RViz:
1. Click **2D Pose Estimate** button (top toolbar)
2. Click on robot location in map
3. Drag arrow to set heading
4. AMCL will converge (green particles → fewer particles)

### 2. Send Navigation Goal

In RViz:
1. Click **Nav2 Goal** button
2. Click desired location on map
3. Drag arrow to set goal orientation
4. Robot will plan and execute path autonomously

### 3. Monitor Navigation

**RViz displays:**
- Green: AMCL particles (localization uncertainty)
- Blue: Local costmap (1m² around robot)
- Gray: Global costmap (full map with inflation radius)
- Red: Local plan
- Cyan: Global plan

**Terminal feedback:**
```bash
ros2 topic echo /robot_pose               # Current estimated pose
ros2 topic echo /amcl_pose_covariance    # Localization uncertainty
ros2 topic echo /local_costmap/costmap   # Local obstacles
```

## ⚙️ Configuration Guide

### AMCL Localization Parameters

Key parameters in `nav2_params.yaml`:
```yaml
amcl:
  ros__parameters:
    laser_min_range: 0.12              # Min LIDAR range
    laser_max_range: 12.0              # Max LIDAR range
    min_particles: 500                 # Min filter particles
    max_particles: 2000                # Max filter particles
    update_min_d: 0.2                  # Distance threshold for update (m)
    update_min_a: 0.2                  # Angle threshold for update (rad)
```

### Controller Configuration

DWB Local Planner settings:
```yaml
controller_server:
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
    max_vel_x: 0.26        # Max linear velocity (m/s)
    max_vel_theta: 1.0     # Max angular velocity (rad/s)
    sim_time: 1.7          # Trajectory simulation horizon (s)
```

### Costmap Configuration

**Local Costmap** (rolling window, 3×3m):
- Resolution: 0.05 m/cell
- Update rate: 5 Hz
- Robot radius inflation: 0.35 m
- Voxel height: 16 layers, 0.05m each

**Global Costmap** (static map):
- Resolution: 0.05 m/cell
- Track unknown: enabled
- Inflation radius: 0.35 m

## 🌍 Gazebo World

### Environment Layout

```
               RED                              GREEN
       (12.3871517,23.0562725)        (23.6751976,22.8605423)
    
               BLUE                            YELLOW
      (12.6005955,11.3363991)         (23.6751575,11.7718639)
       
                           Center (0,0)
                         Robot spawns here
```

### Features
- **4 colored rooms**: Each 6×6m with colored walls
- **Door passages**: Openings at room centers
- **Obstacles**: 3 boxes in central area for navigation challenges
- **Ground plane**: 100×100m field with 0.7 friction
- **Lighting**: Directional sun + shadows enabled

### Spawn Robot at Custom Location

Edit `chair.urdf` spawn command:
```bash
ros2 run gazebo_ros spawn_entity.py -entity robotic_chair -file chair.urdf \
  -x 5.0 -y 5.0 -z 0.1 -R 0 -P 0 -Y 1.57
```

## 🎯 Navigation Tasks

### Example 1: Navigate to Red Room
1. Set 2D pose at origin (0, 0, 0)
2. Click Nav2 Goal at ( 12.3871517 , 23.0562725 )
3. Robot plans around obstacles and navigates

### Example 2: Multi-Room Exploration
1. Localize at origin
2. Send goals sequentially: Red → Green → Blue → Yellow
3. Monitor path quality and recovery behaviors

### Example 3: Test Recovery Behaviors
1. Manually block robot in Gazebo (move obstacles nearby)
2. Send goal that robot cannot reach
3. Observe recovery behaviors: spin, backup, assisted teleop

## 📊 Topic Reference

### Published Topics
| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Wheel odometry |
| `/tf` | `tf2_msgs/TFMessage` | 50 Hz | Transform tree |
| `/scan` | `sensor_msgs/LaserScan` | 5 Hz | LIDAR data |
| `/camera/image_raw` | `sensor_msgs/Image` | 15 Hz | RGB camera |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | IMU measurements |
| `/ground_truth_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 50 Hz | Ground truth |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL initialization |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal |

### Useful Services
```bash
# Trigger global costmap update
ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty

# Trigger local costmap update
ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty

# Reinitialize AMCL
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

## 🛠️ Troubleshooting

### Issue: Robot not localizing (many scattered particles)

**Solution:**
1. Verify LIDAR publishes: `ros2 topic echo /scan`
2. Check map alignment: `ros2 topic echo /map`
3. Manually set 2D pose estimate in RViz
4. Increase `max_particles` in AMCL config

### Issue: Robot gets stuck or oscillates

**Solution:**
1. Increase `sim_time` in DWB planner (more lookahead)
2. Adjust `acc_lim_x` and `acc_lim_theta` (lower = more stable)
3. Check inflation radius isn't too large
4. Verify costmap is updating: `ros2 topic echo /local_costmap/costmap`

### Issue: Navigation goal never completes

**Solution:**
1. Check goal is on free space in map
2. Verify path plan succeeds: `ros2 topic echo /plan`
3. Check `/local_costmap/costmap` for false obstacles
4. Clear costmaps: `ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty`

### Issue: Gazebo crashes or runs slowly

**Solution:**
- Set `real_time_factor: 1` in world file
- Reduce `max_step_size` to 0.0005
- Check system specs: recommend 8GB RAM, multicore CPU
- Use headless mode: `gazebo --headless`

## ⚠️ Known Issues and Difficulties
- Path planning fails in certain corner cases
- Room labeling in RViz can be improved
- The robot freezes in complex maps 

## 📈 Performance Metrics

### Tested Performance (on Intel i7, 16GB RAM)

| Metric | Value | Notes |
|--------|-------|-------|
| **AMCL Update Rate** | 50 Hz | Depends on scan rate |
| **Controller Freq** | 20 Hz | DWB local planner |
| **Planner Freq** | 20 Hz | Navfn global planner |
| **Localization Accuracy** | ±0.15 m | After convergence |
| **Path Planning Time** | 100-500 ms | Depends on map complexity |
| **Simulation Speedup** | ~1.0 | Real-time factor = 1 |

## 🎓 Educational Use

This project is ideal for:
- **ROS 2 Learning**: Complete real-world example
- **Navigation Research**: Testbed for localization/planning algorithms
- **Robotics Courses**: Hands-on autonomous navigation lab
- **Simulation Study**: Gazebo plugin development
- **Parameter Tuning**: Explore Nav2 behavior with different configs

## 📚 Documentation

- [URDF Model Details](#) - Sensor specifications and mounting
- [Nav2 Configuration Guide](#) - Parameter tuning reference
- [World File Documentation](#) - Room layout and obstacles
- [ROS 2 Navigation Concepts](https://docs.nav2.org/)

## 🔮 Future Enhancements

- [ ] Multi-floor navigation (z-axis levels)
- [ ] Semantic segmentation for room understanding
- [ ] Social force model for human-aware navigation
- [ ] Speech command integration
- [ ] Arm manipulation attachment (6-DOF)
- [ ] Battery simulation and charging docks
- [ ] Collision avoidance with moving obstacles
- [ ] Adaptive cost scaling based on terrain type
- [ ] Integration with hospital ROS middleware

## 🤝 Contributing

Contributions welcome! Areas:
- Performance optimization
- Additional sensor plugins
- World environment expansion
- Documentation improvements
- Configuration presets



## 👥 Authors

- **Ilyass Arro**
- **Ayman Adriouch**
- **Abdelgafor hamdaiou**

**Supervised by**: Nimal Elamrani
**Academic Year**: 2025-2026

---



**Last Updated**: January 2026
