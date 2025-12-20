# Motion Player

A ROS 2 package for playing back motion capture data on a humanoid robot (Unitree G1) with simultaneous BVH skeleton visualization in RViz.

## Demo

![Demo](doc/demo.gif)

## Features

- **Motion Playback**: Play pre-recorded motion data (`.pkl` format) on a 29-DOF humanoid robot model
- **Real-time Motion Retargeting**: Receive live mocap data via OSC from [MOVIN TRACIN](https://www.movin3d.com) and retarget to robot in real-time
- **Real-time Visualization**: Simultaneously visualize both the original motion capture skeleton and retargeted robot motion in RViz
- **BVH Visualization**: Display the original BVH motion capture skeleton alongside the robot
- **RViz Integration**: Full visualization in RViz2 with robot model and skeleton markers

## Prerequisites

- **ROS 2**: Humble (tested)
- Required ROS 2 packages:
  - `robot_state_publisher`
  - `rviz2`
  - `xacro`
  - `tf2_ros`

- Required pip packages:
  - `numpy`
  - `scipy`
  - `movin-sdk-python` (required for BVH loading and real-time mode)


## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/MOVIN3D/Motion-Player-ROS
   ```

2. Install MOVIN SDK Python (required for receiving mocap data and real-time retargeting):
   ```bash
   pip install git+https://github.com/MOVIN3D/MOVIN-SDK-Python.git
   ```
   
   For more details about MOVIN SDK, visit: https://github.com/MOVIN3D/MOVIN-SDK-Python

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select motion_player
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Playback Mode (from file)

Launch the motion player with pre-recorded motion files:
```bash
ros2 launch motion_player player.launch.py motion_file:=/path/to/your/motion.pkl bvh_file:=/path/to/your/motion.bvh
```

Or run the node directly:
```bash
ros2 run motion_player motion_player --ros-args -p motion_file:=/path/to/your/motion.pkl
```

### Real-time Mode (live mocap)

Launch the real-time motion player to receive live mocap data via OSC from [MOVIN TRACIN](https://www.movin3d.com):
```bash
ros2 launch motion_player realtime.launch.py
```

This mode enables:
- **Real-time retargeting**: Motion capture data is retargeted to the robot model on-the-fly using [MOVIN SDK Python](https://github.com/MOVIN3D/MOVIN-SDK-Python)
- **Live visualization**: Both the original mocap skeleton and the retargeted robot motion are displayed simultaneously in RViz

With custom parameters:
```bash
ros2 launch motion_player realtime.launch.py port:=11235 human_height:=1.80 skeleton_offset_x:=1.5
```

Or run the node directly with `--realtime` flag:
```bash
ros2 run motion_player motion_player --realtime --ros-args -p port:=11235 -p human_height:=1.80
```

### Launch Arguments

#### player.launch.py (Playback Mode)

| Argument | Default | Description |
|----------|---------|-------------|
| `motion_file` | (required) | Path to the motion pickle file (`.pkl`) |
| `bvh_file` | (required) | Path to the BVH file (`.bvh`)|
| `loop` | `true` | Whether to loop the motion playback |
| `urdf_file` | (package default) | Path to custom URDF file |
| `rviz_config` | (package default) | Path to custom RViz config file |

#### realtime.launch.py (Real-time Mode)

| Argument | Default | Description |
|----------|---------|-------------|
| `port` | `11235` | UDP port to listen for OSC mocap data |
| `robot_type` | `unitree_g1` | Target robot type (`unitree_g1` or `unitree_g1_with_hands`) |
| `human_height` | `1.75` | Human height in meters for scaling |
| `skeleton_offset_x` | `1.0` | X offset to place skeleton beside robot |
| `urdf_file` | (package default) | Path to custom URDF file |
| `rviz_config` | (package default) | Path to custom RViz config file |

## File Formats

### Motion File (`.pkl`)

The motion file contains joint angles that have been **retargeted from motion capture data to the robot URDF**. This file stores the converted motion that can be directly applied to the Unitree G1 robot model.

The pickle file should contain a dictionary with:

```python
{
    'fps': float,           # Frames per second
    'root_pos': np.ndarray, # Root position [n_frames, 3]
    'root_rot': np.ndarray, # Root rotation quaternion (xyzw) [n_frames, 4]
    'dof_pos': np.ndarray,  # Joint positions [n_frames, 29]
}
```

### BVH File (`.bvh`)

The BVH file contains the **original motion capture data** from [MOVIN TRACIN](https://www.movin3d.com) motion capture system.

Standard BVH (Biovision Hierarchy) motion capture format.
The BVH skeleton will be displayed as red spheres (joints) and orange cylinders (bones) in RViz, allowing you to compare the original motion capture with the retargeted robot motion.

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Robot joint states |
| `/skeleton_markers` | `visualization_msgs/MarkerArray` | Skeleton visualization (both modes) |
| `/tf` | TF2 | Robot transforms |

## Project Structure

```
motion_player_ros/
├── CMakeLists.txt
├── package.xml
├── README.md
├── data/
│   ├── test1.bvh              # Example BVH file
│   └── test1.pkl              # Example motion file
├── doc/
│   └── demo.gif               
├── launch/
│   ├── player.launch.py       # Playback mode launch file
│   └── realtime.launch.py     # Real-time mode launch file
├── meshes/                    # Robot mesh files (STL)*
├── rviz/
│   └── robot.rviz             # RViz configuration
├── scripts/
│   └── motion_player.py       # Unified motion player node (supports --realtime flag)
└── urdf/
    └── g1_custom_collision_29dof.urdf  # Robot URDF*
```

## Acknowledgments

This project uses:

- **[MOVIN SDK Python](https://github.com/MOVIN3D/MOVIN-SDK-Python)** - Motion retargeting SDK for real-time mocap to robot conversion
- **[MOVIN TRACIN](https://www.movin3d.com)** - Motion capture system for generating mocap data

The URDF and STL mesh files for the Unitree G1 robot are sourced from [Unitree Robotics](https://github.com/unitreerobotics). Please refer to their repositories for the original robot models and licensing information:

- [unitree_ros](https://github.com/unitreerobotics/unitree_ros) - ROS packages with URDF files for Unitree robots
- [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) - Mujoco simulation for Unitree robots

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

**Note:** The robot URDF and mesh files (`urdf/` and `meshes/` directories) are from Unitree Robotics and may be subject to their own licensing terms.