#!/usr/bin/env python3
"""
Real-time motion player for ROS2.

Receives live motion capture data from MOVIN via OSC/UDP, retargets it to the
Unitree G1 robot, and visualizes both the skeleton and robot in RViz2.

Based on movin_sdk_python examples/mocap_to_robot.py pattern.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np

from movin_sdk_python import Retargeter, MocapReceiver

# Joint names in the order stored in qpos (after root position and orientation)
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


class RealtimeMotionPlayerNode(Node):
    """ROS2 node for real-time motion capture visualization."""

    def __init__(self):
        super().__init__('motion_player2')

        # Declare parameters
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

        # Initialize retargeter (following SDK example pattern)
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

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.skeleton_marker_pub = self.create_publisher(MarkerArray, 'skeleton_markers', 10)

        # State tracking
        self.waiting_for_data = True
        self.warned_missing = 0

        # Timer at ~100Hz to poll for new frames
        timer_period = 1.0 / 100.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Waiting for mocap data on port {port}...')

    def timer_callback(self):
        """Poll for new mocap frames and publish (following SDK example pattern)."""
        # Get latest mocap frame
        frame = self.receiver.get_latest_frame()

        if frame is None:
            return

        if self.waiting_for_data:
            self.waiting_for_data = False
            bone_names = [b["bone_name"] for b in frame["bones"]]
            self.get_logger().info(f'Receiving mocap data! {len(bone_names)} bones')

        bones = frame["bones"]

        # Process mocap frame (following SDK example)
        mocap_data = self.retargeter.process_mocap_frame(bones)

        # Check for required bones
        missing = sorted(self.required_bones.difference(mocap_data.keys()))
        if missing:
            if self.warned_missing < 5:
                self.warned_missing += 1
                self.get_logger().warn(f'Missing {len(missing)} required bones: {missing}')
            return

        # Filter to only required bones for retargeting (following SDK example)
        filtered_mocap_data = {k: mocap_data[k] for k in self.required_bones if k in mocap_data}

        # Retarget (following SDK example - no offset_to_ground argument)
        qpos = self.retargeter.retarget(filtered_mocap_data)

        now = self.get_clock().now().to_msg()

        # Publish robot state
        self.publish_robot_state(qpos, now)

        # Publish skeleton markers using ALL bones (not filtered)
        self.publish_skeleton_markers(mocap_data, bones, now)

    def publish_robot_state(self, qpos, stamp):
        """Publish robot joint states and root TF."""
        # Extract root position and orientation
        root_pos = qpos[:3]
        root_rot = qpos[3:7]  # (w, x, y, z) format from mujoco
        dof_pos = qpos[7:]

        # Publish TF for pelvis (root)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'pelvis'

        t.transform.translation.x = float(root_pos[0])
        t.transform.translation.y = float(root_pos[1])
        t.transform.translation.z = float(root_pos[2])

        # qpos quaternion is (w, x, y, z), ROS TF expects (x, y, z, w)
        t.transform.rotation.x = float(root_rot[1])
        t.transform.rotation.y = float(root_rot[2])
        t.transform.rotation.z = float(root_rot[3])
        t.transform.rotation.w = float(root_rot[0])

        self.tf_broadcaster.sendTransform(t)

        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = JOINT_NAMES
        joint_state.position = dof_pos.tolist()

        self.joint_state_pub.publish(joint_state)

    def publish_skeleton_markers(self, mocap_data, bones, stamp):
        """Publish skeleton visualization as MarkerArray."""
        marker_array = MarkerArray()
        marker_id = 0

        # Build bone positions from mocap_data and parent indices from bones
        bone_positions = {}  # bone_idx -> position
        bone_parents = {}    # bone_idx -> parent_idx
        bone_idx_to_name = {}  # bone_idx -> bone_name

        for bone in bones:
            bone_name = bone["bone_name"]
            bone_idx = bone["bone_index"]
            parent_idx = bone["parent_index"]

            bone_parents[bone_idx] = parent_idx
            bone_idx_to_name[bone_idx] = bone_name

            # Get position from mocap_data (already processed with FK)
            if bone_name in mocap_data:
                pos = np.array(mocap_data[bone_name][0])
                bone_positions[bone_idx] = pos

        # Create sphere markers for each joint
        for bone_idx, pos in bone_positions.items():
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'world'
            marker.ns = 'skeleton_joints'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Apply offset to show skeleton beside robot
            marker.pose.position.x = float(pos[0] + self.skeleton_offset_x)
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0

            # Sphere size
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03

            # Red color for joints
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 1.0

            marker_array.markers.append(marker)
            marker_id += 1

        # Create cylinder markers for bones (connecting joint to parent)
        for bone_idx, pos in bone_positions.items():
            parent_idx = bone_parents.get(bone_idx, -1)

            # Skip if no parent, parent not in positions, or parent is root (index 0)
            if parent_idx <= 0 or parent_idx not in bone_positions:
                continue

            start = bone_positions[parent_idx].copy()
            end = pos.copy()

            # Apply offset
            start[0] += self.skeleton_offset_x
            end[0] += self.skeleton_offset_x

            # Calculate cylinder center and orientation
            center = (start + end) / 2.0
            diff = end - start
            length = np.linalg.norm(diff)

            if length < 1e-6:
                continue

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

            # Cylinder size
            marker.scale.x = 0.015
            marker.scale.y = 0.015
            marker.scale.z = float(length)

            # Orange color for bones
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
            marker_id += 1

        self.skeleton_marker_pub.publish(marker_array)

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Stopping MocapReceiver...')
        self.receiver.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeMotionPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shutdown by signal handler


if __name__ == '__main__':
    main()
