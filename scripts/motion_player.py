#!/usr/bin/env python3
"""
Motion player for ROS2.

Supports two modes:
  1. Playback mode (default): Plays pre-recorded motion from pickle/BVH files
  2. Realtime mode (--realtime): Receives live mocap data via OSC/UDP

Usage:
  ros2 run motion_player motion_player --ros-args -p motion_file:=/path/to/motion.pkl
  ros2 run motion_player motion_player --realtime --ros-args -p port:=11235
"""

import os
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np

# Joint names in the order stored in the motion data
JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


class MotionPlayerNode(Node):
    """Unified ROS2 node for motion playback and real-time mocap visualization."""

    def __init__(self, realtime: bool = False):
        super().__init__('motion_player')
        self.realtime = realtime

        # Common publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.skeleton_marker_pub = self.create_publisher(MarkerArray, 'skeleton_markers', 10)

        if self.realtime:
            self._init_realtime()
            self.timer = self.create_timer(1.0 / 100.0, self._realtime_callback)
        else:
            self._init_playback()
            if hasattr(self, 'fps') and self.fps > 0:
                self.timer = self.create_timer(1.0 / self.fps, self._playback_callback)

    # -------------------------------------------------------------------------
    # Playback mode initialization and callback
    # -------------------------------------------------------------------------
    def _init_playback(self):
        """Initialize playback mode from pickle/BVH files."""
        import pickle
        from movin_sdk_python import load_bvh_file

        # Declare parameters for playback mode
        self.declare_parameter('motion_file', '')
        self.declare_parameter('bvh_file', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('skeleton_offset_x', 1.0)
        self.declare_parameter('human_height', 1.75)

        motion_file = self.get_parameter('motion_file').get_parameter_value().string_value
        bvh_file_param = self.get_parameter('bvh_file').get_parameter_value().string_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.skeleton_offset_x = self.get_parameter('skeleton_offset_x').get_parameter_value().double_value
        human_height = self.get_parameter('human_height').get_parameter_value().double_value

        if not motion_file:
            self.get_logger().error('No motion_file specified!')
            return

        # Load motion data
        with open(motion_file, 'rb') as f:
            self.motion_data = pickle.load(f)

        self.fps = self.motion_data['fps']
        self.root_pos = self.motion_data['root_pos']      # [n_frames, 3]
        self.root_rot = self.motion_data['root_rot']      # [n_frames, 4] xyzw
        self.dof_pos = self.motion_data['dof_pos']        # [n_frames, 29]
        self.n_frames = len(self.root_pos)
        self.current_frame = 0

        self.get_logger().info(f'Loaded motion with {self.n_frames} frames at {self.fps} FPS')

        # Load BVH data if available
        self.bvh_frames = None
        self.bvh_bone_names = None
        self.bvh_bone_parents = None

        # Use explicit bvh_file parameter, or auto-detect from motion_file
        if bvh_file_param:
            bvh_file = bvh_file_param
        else:
            bvh_file = motion_file.replace('.pkl', '.bvh')

        if os.path.exists(bvh_file):
            # load_bvh_file returns (frames, human_height, parents, bones)
            self.bvh_frames, _, self.bvh_bone_parents, self.bvh_bone_names = load_bvh_file(
                bvh_file, human_height=human_height
            )
            self.get_logger().info(f'Loaded BVH with {len(self.bvh_frames)} frames, {len(self.bvh_bone_names)} bones')
        else:
            self.get_logger().warn(f'No BVH file found at {bvh_file}')

    def _playback_callback(self):
        """Timer callback for playback mode."""
        if self.current_frame >= self.n_frames:
            if self.loop:
                self.current_frame = 0
            else:
                self.get_logger().info('Motion playback complete')
                self.timer.cancel()
                return

        now = self.get_clock().now().to_msg()

        # Get current frame data
        root_pos = self.root_pos[self.current_frame]
        root_rot_xyzw = self.root_rot[self.current_frame]
        dof_pos = self.dof_pos[self.current_frame]

        # Publish robot state (convert xyzw to wxyz for consistency)
        root_rot_wxyz = np.array([root_rot_xyzw[3], root_rot_xyzw[0], root_rot_xyzw[1], root_rot_xyzw[2]])
        self._publish_robot_state(root_pos, root_rot_wxyz, dof_pos, now)

        # Publish BVH markers if available
        if self.bvh_frames is not None:
            # Use modulo to handle frame count mismatch between BVH and PKL
            bvh_frame_idx = self.current_frame % len(self.bvh_frames)
            frame_data = self.bvh_frames[bvh_frame_idx]
            self._publish_skeleton_markers(frame_data, now, self.bvh_bone_names, self.bvh_bone_parents)

        self.current_frame += 1

    # -------------------------------------------------------------------------
    # Realtime mode initialization and callback
    # -------------------------------------------------------------------------
    def _init_realtime(self):
        """Initialize realtime mode with mocap receiver."""
        from movin_sdk_python import Retargeter, MocapReceiver

        # Declare parameters for realtime mode
        self.declare_parameter('port', 11235)
        self.declare_parameter('robot_type', 'unitree_g1')
        self.declare_parameter('human_height', 1.75)
        self.declare_parameter('skeleton_offset_x', 1.0)

        # Get parameters
        port = self.get_parameter('port').get_parameter_value().integer_value
        robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        human_height = self.get_parameter('human_height').get_parameter_value().double_value
        self.skeleton_offset_x = self.get_parameter('skeleton_offset_x').get_parameter_value().double_value

        self.get_logger().info(f'Initializing Retargeter for {robot_type} with human_height={human_height}')

        # Initialize retargeter
        self.retargeter = Retargeter(
            robot_type=robot_type,
            human_height=human_height,
            verbose=False,
        )
        self.required_bones = self.retargeter.get_required_bones()

        # Initialize mocap receiver
        self.receiver = MocapReceiver(port=port)
        self.receiver.start()
        self.get_logger().info(f'MocapReceiver started on port {port}')

        # State tracking
        self.waiting_for_data = True
        self.warned_missing = 0

        self.get_logger().info(f'Waiting for mocap data on port {port}...')

    def _realtime_callback(self):
        """Timer callback for realtime mode - poll for mocap frames."""
        # Get latest mocap frame
        frame = self.receiver.get_latest_frame()

        if frame is None:
            return

        if self.waiting_for_data:
            self.waiting_for_data = False
            bone_names = [b["bone_name"] for b in frame["bones"]]
            self.get_logger().info(f'Receiving mocap data! {len(bone_names)} bones')

        bones = frame["bones"]

        # Process mocap frame
        mocap_data = self.retargeter.process_mocap_frame(bones)

        # Check for required bones
        missing = sorted(self.required_bones.difference(mocap_data.keys()))
        if missing:
            if self.warned_missing < 5:
                self.warned_missing += 1
                self.get_logger().warn(f'Missing {len(missing)} required bones: {missing}')
            return

        # Filter to only required bones for retargeting
        filtered_mocap_data = {k: mocap_data[k] for k in self.required_bones if k in mocap_data}

        # Retarget
        qpos = self.retargeter.retarget(filtered_mocap_data)

        now = self.get_clock().now().to_msg()

        # Extract root position and orientation
        root_pos = qpos[:3]
        root_rot = qpos[3:7]  # (w, x, y, z) format from mujoco
        dof_pos = qpos[7:]

        # Publish robot state
        self._publish_robot_state(root_pos, root_rot, dof_pos, now)

        # Publish skeleton markers with bone hierarchy from mocap frame
        bone_names = [b["bone_name"] for b in bones]
        bone_parents = {b["bone_index"]: b["parent_index"] for b in bones}
        self._publish_skeleton_markers(mocap_data, now, bone_names, bone_parents)

    # -------------------------------------------------------------------------
    # Common methods
    # -------------------------------------------------------------------------
    def _publish_skeleton_markers(self, frame_data: dict, stamp, bone_names, bone_parents):
        """Publish skeleton visualization as MarkerArray.
        
        Args:
            frame_data: Dictionary mapping bone_name -> [position, orientation]
                       Position is [x, y, z], orientation is quaternion
            stamp: ROS timestamp
            bone_names: List of bone names (ordered by index) or None
            bone_parents: Parent indices - either np.array (BVH) or dict {idx: parent_idx} (realtime)
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Build bone positions dict
        bone_positions = {}
        for bone_name in frame_data.keys():
            pos = np.array(frame_data[bone_name][0])
            bone_positions[bone_name] = pos

        # Create sphere markers for each joint
        for bone_name, pos in bone_positions.items():
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'world'
            marker.ns = 'skeleton_joints'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(pos[0] + self.skeleton_offset_x)
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 1.0

            marker_array.markers.append(marker)
            marker_id += 1

        # Create cylinder markers for bones using parent hierarchy
        if bone_names is not None and bone_parents is not None:
            for joint_idx in range(1, len(bone_names)):
                bone_name = bone_names[joint_idx]
                
                # Get parent index (handle both np.array and dict)
                if isinstance(bone_parents, dict):
                    parent_idx = bone_parents.get(joint_idx, -1)
                else:
                    parent_idx = int(bone_parents[joint_idx])
                
                if parent_idx < 0 or parent_idx >= len(bone_names):
                    continue
                
                parent_name = bone_names[parent_idx]
                
                if bone_name not in bone_positions or parent_name not in bone_positions:
                    continue

                start = bone_positions[parent_name].copy()
                end = bone_positions[bone_name].copy()

                start[0] += self.skeleton_offset_x
                end[0] += self.skeleton_offset_x

                self._add_bone_marker(marker_array, marker_id, start, end, stamp)
                marker_id += 1

        self.skeleton_marker_pub.publish(marker_array)

    def _publish_robot_state(self, root_pos, root_rot_wxyz, dof_pos, stamp):
        """Publish robot joint states and root TF.
        
        Args:
            root_pos: [x, y, z] root position
            root_rot_wxyz: [w, x, y, z] root quaternion
            dof_pos: joint positions array
            stamp: ROS timestamp
        """
        # Publish TF for pelvis (root)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'pelvis'

        t.transform.translation.x = float(root_pos[0])
        t.transform.translation.y = float(root_pos[1])
        t.transform.translation.z = float(root_pos[2])

        # ROS TF expects (x, y, z, w)
        t.transform.rotation.x = float(root_rot_wxyz[1])
        t.transform.rotation.y = float(root_rot_wxyz[2])
        t.transform.rotation.z = float(root_rot_wxyz[3])
        t.transform.rotation.w = float(root_rot_wxyz[0])

        self.tf_broadcaster.sendTransform(t)

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = JOINT_NAMES
        joint_state.position = dof_pos.tolist() if hasattr(dof_pos, 'tolist') else list(dof_pos)

        self.joint_state_pub.publish(joint_state)

    def _add_bone_marker(self, marker_array, marker_id, start, end, stamp):
        """Add a cylinder marker connecting two joints."""
        center = (start + end) / 2.0
        diff = end - start
        length = np.linalg.norm(diff)

        if length < 1e-6:
            return

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'world'
        marker.ns = 'skeleton_bones'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])

        # Calculate quaternion to orient cylinder along bone
        direction = diff / length
        z_axis = np.array([0.0, 0.0, 1.0])

        rot_axis = np.cross(z_axis, direction)
        rot_axis_norm = np.linalg.norm(rot_axis)

        if rot_axis_norm < 1e-6:
            if direction[2] > 0:
                marker.pose.orientation.w = 1.0
            else:
                marker.pose.orientation.x = 1.0
                marker.pose.orientation.w = 0.0
        else:
            rot_axis = rot_axis / rot_axis_norm
            angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
            marker.pose.orientation.x = float(rot_axis[0] * np.sin(angle / 2))
            marker.pose.orientation.y = float(rot_axis[1] * np.sin(angle / 2))
            marker.pose.orientation.z = float(rot_axis[2] * np.sin(angle / 2))
            marker.pose.orientation.w = float(np.cos(angle / 2))

        marker.scale.x = 0.015
        marker.scale.y = 0.015
        marker.scale.z = float(length)

        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker_array.markers.append(marker)

    def destroy_node(self):
        """Clean up resources."""
        if self.realtime and hasattr(self, 'receiver'):
            self.get_logger().info('Stopping MocapReceiver...')
            self.receiver.stop()
        super().destroy_node()


def main(args=None):
    # Parse --realtime argument before ROS args
    parser = argparse.ArgumentParser(description='Motion player for ROS2')
    parser.add_argument('--realtime', action='store_true',
                        help='Use live mocap instead of file playback')
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MotionPlayerNode(realtime=parsed.realtime)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
