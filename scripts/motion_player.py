#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import pickle
import numpy as np

from bvh_reader import BVHReader

# Joint names in the order stored in the pickle file
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
    def __init__(self):
        super().__init__('motion_player')
        
        # Declare parameters
        self.declare_parameter('motion_file', '')
        self.declare_parameter('bvh_file', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('bvh_scale', 1.0)  # Scale factor for BVH positions (already in meters)
        self.declare_parameter('bvh_offset_x', 1.0)  # X offset to place BVH beside robot
        
        motion_file = self.get_parameter('motion_file').get_parameter_value().string_value
        bvh_file_param = self.get_parameter('bvh_file').get_parameter_value().string_value
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.bvh_scale = self.get_parameter('bvh_scale').get_parameter_value().double_value
        self.bvh_offset_x = self.get_parameter('bvh_offset_x').get_parameter_value().double_value
        
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
        self.bvh_data = None
        self.bvh_global_pos = None
        self.bvh_global_quats = None
        
        # Use explicit bvh_file parameter, or auto-detect from motion_file
        if bvh_file_param:
            bvh_file = bvh_file_param
        else:
            bvh_file = motion_file.replace('.pkl', '.bvh')
        
        if os.path.exists(bvh_file):
            reader = BVHReader()
            self.bvh_data = reader.read(bvh_file)
            self.bvh_global_pos, self.bvh_global_quats = reader.compute_global_transforms(self.bvh_data)
            
            # Apply Y-up to Z-up coordinate transformation
            rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float32)
            # Transform all positions: (T, J, 3) @ (3, 3).T
            self.bvh_global_pos = np.einsum('tji,ki->tjk', self.bvh_global_pos, rotation_matrix)
            
            self.get_logger().info(f'Loaded BVH with {self.bvh_data.num_frames} frames, {self.bvh_data.num_joints} joints')
        else:
            self.get_logger().warn(f'No BVH file found at {bvh_file}')
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bvh_marker_pub = self.create_publisher(MarkerArray, 'bvh_markers', 10)
        
        # Timer to publish at motion FPS
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        if self.current_frame >= self.n_frames:
            if self.loop:
                self.current_frame = 0
            else:
                self.get_logger().info('Motion playback complete')
                self.timer.cancel()
                return
        
        now = self.get_clock().now().to_msg()
        
        # Publish TF for pelvis (root)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'pelvis'
        
        t.transform.translation.x = float(self.root_pos[self.current_frame, 0])
        t.transform.translation.y = float(self.root_pos[self.current_frame, 1])
        t.transform.translation.z = float(self.root_pos[self.current_frame, 2])
        
        # root_rot is in xyzw format
        t.transform.rotation.x = float(self.root_rot[self.current_frame, 0])
        t.transform.rotation.y = float(self.root_rot[self.current_frame, 1])
        t.transform.rotation.z = float(self.root_rot[self.current_frame, 2])
        t.transform.rotation.w = float(self.root_rot[self.current_frame, 3])
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = JOINT_NAMES
        joint_state.position = self.dof_pos[self.current_frame].tolist()
        
        self.joint_state_pub.publish(joint_state)
        
        # Publish BVH markers if available
        if self.bvh_data is not None:
            self.publish_bvh_markers(now)
        
        self.current_frame += 1
    
    def publish_bvh_markers(self, stamp):
        """Publish BVH skeleton as MarkerArray (spheres for joints, cylinders for bones)."""
        marker_array = MarkerArray()
        
        # Use modulo to handle frame count mismatch between BVH and PKL
        bvh_frame = self.current_frame % self.bvh_data.num_frames
        
        # Get joint positions for this frame
        positions = self.bvh_global_pos[bvh_frame]  # (J, 3)
        
        marker_id = 0
        
        # Create sphere markers for each joint
        for joint_idx, bone_name in enumerate(self.bvh_data.bones):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'world'
            marker.ns = 'bvh_joints'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Apply scale and offset
            marker.pose.position.x = float(positions[joint_idx, 0] * self.bvh_scale + self.bvh_offset_x)
            marker.pose.position.y = float(positions[joint_idx, 1] * self.bvh_scale)
            marker.pose.position.z = float(positions[joint_idx, 2] * self.bvh_scale)
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
        for joint_idx in range(1, self.bvh_data.num_joints):
            parent_idx = self.bvh_data.parents[joint_idx]
            if parent_idx < 0:
                continue
            
            # Get start and end positions
            start = positions[parent_idx] * self.bvh_scale
            end = positions[joint_idx] * self.bvh_scale
            
            # Apply offset
            start[0] += self.bvh_offset_x
            end[0] += self.bvh_offset_x
            
            # Calculate cylinder center and orientation
            center = (start + end) / 2.0
            diff = end - start
            length = np.linalg.norm(diff)
            
            if length < 1e-6:
                continue
            
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'world'
            marker.ns = 'bvh_bones'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            
            # Calculate quaternion to orient cylinder along bone
            # Cylinder default axis is Z, we need to rotate to align with diff
            direction = diff / length
            z_axis = np.array([0.0, 0.0, 1.0])
            
            # Rotation axis and angle
            rot_axis = np.cross(z_axis, direction)
            rot_axis_norm = np.linalg.norm(rot_axis)
            
            if rot_axis_norm < 1e-6:
                # Parallel to Z axis
                if direction[2] > 0:
                    marker.pose.orientation.w = 1.0
                else:
                    marker.pose.orientation.x = 1.0
                    marker.pose.orientation.w = 0.0
            else:
                rot_axis = rot_axis / rot_axis_norm
                angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
                # Quaternion from axis-angle
                marker.pose.orientation.x = float(rot_axis[0] * np.sin(angle / 2))
                marker.pose.orientation.y = float(rot_axis[1] * np.sin(angle / 2))
                marker.pose.orientation.z = float(rot_axis[2] * np.sin(angle / 2))
                marker.pose.orientation.w = float(np.cos(angle / 2))
            
            # Cylinder size (narrow)
            marker.scale.x = 0.015  # diameter
            marker.scale.y = 0.015  # diameter
            marker.scale.z = float(length)  # height
            
            # Orange color for bones
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        self.bvh_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
