"""
Simple BVH file reader that outputs global positions and orientations.

This module provides a clean interface for reading BVH motion capture files
and computing global joint positions and orientations.
"""

import re
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
from scipy.spatial.transform import Rotation as R


@dataclass
class BVHData:
    """Container for BVH animation data."""
    bones: List[str]              # Joint names
    parents: np.ndarray           # Parent indices for each joint
    offsets: np.ndarray           # Local joint offsets (J, 3)
    local_quats: np.ndarray       # Local quaternions (T, J, 4) in wxyz format
    local_pos: np.ndarray         # Local positions (T, J, 3)
    frame_time: float             # Time per frame in seconds
    
    @property
    def num_frames(self) -> int:
        return self.local_pos.shape[0]
    
    @property
    def num_joints(self) -> int:
        return len(self.bones)


class BVHReader:
    """
    Simple BVH file reader.
    
    Usage:
        reader = BVHReader()
        data = reader.read("motion.bvh")
        global_pos, global_quats = reader.compute_global_transforms(data)
    """
    
    CHANNEL_MAP = {'Xrotation': 'x', 'Yrotation': 'y', 'Zrotation': 'z'}
    
    def read(self, filename: str, start: int = None, end: int = None) -> BVHData:
        """
        Read a BVH file and extract animation data.
        
        Args:
            filename: Path to BVH file
            start: Start frame (optional)
            end: End frame (optional)
            
        Returns:
            BVHData object containing the animation data
        """
        with open(filename, "r") as f:
            lines = f.readlines()
        
        # Parse hierarchy (now also returns per-joint channel info)
        names, offsets, parents, joint_channels, rot_order = self._parse_hierarchy(lines)
        
        # Parse motion data
        local_pos, rotations, frame_time = self._parse_motion(
            lines, len(names), offsets, joint_channels, start, end
        )
        
        # Convert euler angles to quaternions
        local_quats = self._euler_to_quat(np.radians(rotations), rot_order)
        local_quats = self._remove_quat_discontinuities(local_quats)
        
        return BVHData(
            bones=names,
            parents=parents,
            offsets=offsets,
            local_quats=local_quats,
            local_pos=local_pos,
            frame_time=frame_time
        )
    
    def compute_global_transforms(
        self, data: BVHData
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute global positions and quaternions using forward kinematics.
        
        Args:
            data: BVHData object from read()
            
        Returns:
            Tuple of (global_positions, global_quaternions)
            - global_positions: (T, J, 3) array
            - global_quaternions: (T, J, 4) array in wxyz format
        """
        return self._forward_kinematics(
            data.local_quats, data.local_pos, data.parents
        )
    
    def _parse_hierarchy(
        self, lines: List[str]
    ) -> Tuple[List[str], np.ndarray, np.ndarray, List[int], str]:
        """Parse the HIERARCHY section of a BVH file.
        
        Returns:
            Tuple of (names, offsets, parents, joint_channels, rotation_order)
            - joint_channels: list of channel counts per joint (3 or 6)
            - rotation_order: euler rotation order (e.g., 'yxz')
        """
        names = []
        offsets_list = []
        parents_list = []
        joint_channels = []  # Track channels per joint
        active = -1
        end_site = False
        rot_order = None
        
        for line in lines:
            line = line.strip()
            
            if line.startswith("HIERARCHY") or line.startswith("MOTION"):
                continue
            if line == "{":
                continue
                
            if line == "}":
                if end_site:
                    end_site = False
                else:
                    active = parents_list[active]
                continue
            
            # ROOT joint
            match = re.match(r"ROOT\s+(\w+)", line)
            if match:
                names.append(match.group(1))
                offsets_list.append([0.0, 0.0, 0.0])
                parents_list.append(active)
                active = len(parents_list) - 1
                continue
            
            # JOINT
            match = re.match(r"JOINT\s+(\w+)", line)
            if match:
                names.append(match.group(1))
                offsets_list.append([0.0, 0.0, 0.0])
                parents_list.append(active)
                active = len(parents_list) - 1
                continue
            
            # OFFSET
            match = re.match(r"OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
            if match and not end_site:
                offsets_list[active] = [float(x) for x in match.groups()]
                continue
            
            # CHANNELS - track per joint
            match = re.match(r"CHANNELS\s+(\d+)", line)
            if match:
                num_channels = int(match.group(1))
                joint_channels.append(num_channels)
                
                # Extract rotation order from channels
                if rot_order is None:
                    parts = line.split()[2:]  # Skip "CHANNELS N"
                    rot_parts = [p for p in parts if p in self.CHANNEL_MAP]
                    if rot_parts:
                        rot_order = "".join([self.CHANNEL_MAP[p] for p in rot_parts])
                continue
            
            # End Site
            if line.startswith("End Site"):
                end_site = True
                continue
            
            # Stop at MOTION section
            if line.startswith("Frames:"):
                break
        
        return (
            names,
            np.array(offsets_list, dtype=np.float32),
            np.array(parents_list, dtype=np.int32),
            joint_channels,
            rot_order or 'zyx'
        )
    
    def _parse_motion(
        self,
        lines: List[str],
        num_joints: int,
        offsets: np.ndarray,
        joint_channels: List[int],
        start: int = None,
        end: int = None
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Parse the MOTION section of a BVH file.
        
        Args:
            lines: File lines
            num_joints: Number of joints
            offsets: Joint offsets (J, 3)
            joint_channels: Channel count per joint (3 or 6)
            start: Start frame (optional)
            end: End frame (optional)
            
        Returns:
            Tuple of (positions, rotations, frame_time)
        """
        frame_time = 0.0
        num_frames = 0
        positions = None
        rotations = None
        frame_idx = 0
        data_frame_idx = 0
        in_motion = False
        
        for line in lines:
            line = line.strip()
            
            # Frames count
            match = re.match(r"Frames:\s*(\d+)", line)
            if match:
                if start is not None and end is not None:
                    num_frames = end - start
                else:
                    num_frames = int(match.group(1))
                positions = np.tile(offsets, (num_frames, 1, 1))
                rotations = np.zeros((num_frames, num_joints, 3), dtype=np.float32)
                in_motion = True
                continue
            
            # Frame time
            match = re.match(r"Frame Time:\s*([\d\.]+)", line)
            if match:
                frame_time = float(match.group(1))
                continue
            
            # Motion data
            if in_motion and line and not line.startswith("Frame"):
                # Skip frames outside range
                if start is not None and end is not None:
                    if frame_idx < start or frame_idx >= end:
                        frame_idx += 1
                        continue
                
                values = [float(x) for x in line.split()]
                
                # Parse per-joint based on channel count
                idx = 0
                for joint_idx, ch_count in enumerate(joint_channels):
                    if ch_count == 6:
                        # Position + Rotation
                        positions[data_frame_idx, joint_idx] = values[idx:idx+3]
                        rotations[data_frame_idx, joint_idx] = values[idx+3:idx+6]
                        idx += 6
                    elif ch_count == 3:
                        # Rotation only
                        rotations[data_frame_idx, joint_idx] = values[idx:idx+3]
                        idx += 3
                
                frame_idx += 1
                data_frame_idx += 1
        
        return positions, rotations, frame_time
    
    def _euler_to_quat(self, euler: np.ndarray, order: str = 'zyx') -> np.ndarray:
        """
        Convert euler angles to quaternions.
        
        Args:
            euler: (T, J, 3) euler angles in radians
            order: rotation order string (e.g., 'zyx', 'xyz')
            
        Returns:
            (T, J, 4) quaternions in wxyz format
        """
        axis_map = {
            'x': np.array([1, 0, 0], dtype=np.float32),
            'y': np.array([0, 1, 0], dtype=np.float32),
            'z': np.array([0, 0, 1], dtype=np.float32)
        }
        
        def angle_axis_to_quat(angle, axis):
            c = np.cos(angle / 2.0)[..., np.newaxis]
            s = np.sin(angle / 2.0)[..., np.newaxis]
            return np.concatenate([c, s * axis], axis=-1)
        
        q0 = angle_axis_to_quat(euler[..., 0], axis_map[order[0]])
        q1 = angle_axis_to_quat(euler[..., 1], axis_map[order[1]])
        q2 = angle_axis_to_quat(euler[..., 2], axis_map[order[2]])
        
        return self._quat_mul(q0, self._quat_mul(q1, q2))
    
    def _quat_mul(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Multiply quaternions (wxyz format)."""
        x0, x1, x2, x3 = x[..., 0:1], x[..., 1:2], x[..., 2:3], x[..., 3:4]
        y0, y1, y2, y3 = y[..., 0:1], y[..., 1:2], y[..., 2:3], y[..., 3:4]
        
        return np.concatenate([
            y0 * x0 - y1 * x1 - y2 * x2 - y3 * x3,
            y0 * x1 + y1 * x0 - y2 * x3 + y3 * x2,
            y0 * x2 + y1 * x3 + y2 * x0 - y3 * x1,
            y0 * x3 - y1 * x2 + y2 * x1 + y3 * x0
        ], axis=-1)
    
    def _quat_mul_vec(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Rotate vector by quaternion (wxyz format)."""
        t = 2.0 * np.cross(q[..., 1:], v)
        return v + q[..., 0:1] * t + np.cross(q[..., 1:], t)
    
    def _forward_kinematics(
        self,
        local_quats: np.ndarray,
        local_pos: np.ndarray,
        parents: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Perform forward kinematics to compute global transforms.
        
        Args:
            local_quats: (T, J, 4) local quaternions in wxyz format
            local_pos: (T, J, 3) local positions
            parents: (J,) parent indices
            
        Returns:
            Tuple of (global_positions, global_quaternions)
        """
        global_pos = [local_pos[..., :1, :]]
        global_rot = [local_quats[..., :1, :]]
        
        for i in range(1, len(parents)):
            parent_idx = parents[i]
            pos = self._quat_mul_vec(
                global_rot[parent_idx],
                local_pos[..., i:i+1, :]
            ) + global_pos[parent_idx]
            rot = self._quat_mul(global_rot[parent_idx], local_quats[..., i:i+1, :])
            
            global_pos.append(pos)
            global_rot.append(rot)
        
        return (
            np.concatenate(global_pos, axis=-2),
            np.concatenate(global_rot, axis=-2)
        )
    
    def _remove_quat_discontinuities(self, quats: np.ndarray) -> np.ndarray:
        """Remove quaternion sign flips for smooth interpolation."""
        quats = quats.copy()
        quats_inv = -quats
        
        for i in range(1, quats.shape[0]):
            dot = np.sum(quats[i-1:i] * quats[i:i+1], axis=-1)
            dot_inv = np.sum(quats[i-1:i] * quats_inv[i:i+1], axis=-1)
            mask = (dot < dot_inv)[..., np.newaxis]
            quats[i] = mask * quats_inv[i] + (1.0 - mask) * quats[i]
        
        return quats


def load_bvh_global(
    bvh_file: str,
    apply_coord_transform: bool = True
) -> List[Dict]:
    """
    Load a BVH file and return global positions and orientations per frame.
    
    Args:
        bvh_file: Path to BVH file
        apply_coord_transform: Whether to apply coordinate transformation (Y-up to Z-up)
        
    Returns:
        List of dictionaries, each containing:
            {bone_name: [position (3,), orientation (4,)], ...}
        Orientations are quaternions in wxyz format.
    """
    reader = BVHReader()
    data = reader.read(bvh_file)
    global_pos, global_quats = reader.compute_global_transforms(data)
    
    # Coordinate transformation: Y-up to Z-up (rotate -90 degrees around X)
    if apply_coord_transform:
        rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float32)
        rotation_quat = R.from_matrix(rotation_matrix).as_quat(scalar_first=True)
    
    # Build per-frame dictionaries
    frames = []
    for frame_idx in range(data.num_frames):
        result = {}
        for joint_idx, bone in enumerate(data.bones):
            if apply_coord_transform:
                # Transform position
                position = global_pos[frame_idx, joint_idx] @ rotation_matrix.T
                # Transform orientation
                orientation = reader._quat_mul(rotation_quat, global_quats[frame_idx, joint_idx])
            else:
                position = global_pos[frame_idx, joint_idx]
                orientation = global_quats[frame_idx, joint_idx]
            
            result[bone] = [position, orientation]
        
        frames.append(result)
    
    return frames


if __name__ == "__main__":
    reader = BVHReader()
    data = reader.read("test1.bvh")

    print(data.bones)        # Joint names
    print(data.parents)      # Parent indices
    print(data.local_quats.shape)  # (T, J, 4) local quaternions
    print(data.local_pos.shape)    # (T, J, 3) local positions
    print(data.frame_time)   # Time per frame


    global_pos, global_quats = reader.compute_global_transforms(data)

    print(global_pos.shape)
    print(global_quats.shape)