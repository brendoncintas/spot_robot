#!/usr/bin/env python3

from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
import struct
import rclpy
from rclpy.node import Node
import numpy as np
from bosdyn.client.frame_helpers import *
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import local_grid_pb2
import bosdyn.client
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def expand_data_by_rle_count(local_grid_proto, data_type=np.int16):
    """Expand local grid data to full bytes data using the RLE count."""
    cells_pz = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
    cells_pz_full = []
    # For each value of rle_counts, we expand the cell data at the matching index
    # to have that many repeated, consecutive values.
    for i in range(0, len(local_grid_proto.local_grid.rle_counts)):
        for j in range(0, local_grid_proto.local_grid.rle_counts[i]):
            cells_pz_full.append(cells_pz[i])
    return np.array(cells_pz_full)

def get_numpy_data_type(local_grid_proto):
    """Convert the cell format of the local grid proto to a numpy data type."""
    if local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT16:
        return np.uint16
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT16:
        return np.int16
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_UINT8:
        return np.uint8
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_INT8:
        return np.int8
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT64:
        return np.float64
    elif local_grid_proto.cell_format == local_grid_pb2.LocalGrid.CELL_FORMAT_FLOAT32:
        return np.float32
    else:
        return None

def unpack_grid(local_grid_proto):
    """Unpack the local grid proto."""
    # Determine the data type for the bytes data.
    data_type = get_numpy_data_type(local_grid_proto.local_grid)
    if data_type is None:
        print('Cannot determine the dataformat for the local grid.')
        return None
    # Decode the local grid.
    if local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RAW:
        full_grid = np.frombuffer(local_grid_proto.local_grid.data, dtype=data_type)
    elif local_grid_proto.local_grid.encoding == local_grid_pb2.LocalGrid.ENCODING_RLE:
        full_grid = expand_data_by_rle_count(local_grid_proto, data_type=data_type)
    else:
        # Return nothing if there is no encoding type set.
        return None
    # Apply the offset and scaling to the local grid.
    if local_grid_proto.local_grid.cell_value_scale == 0:
        return full_grid
    full_grid_float = full_grid.astype(np.float64)
    full_grid_float *= local_grid_proto.local_grid.cell_value_scale
    full_grid_float += local_grid_proto.local_grid.cell_value_offset
    return full_grid_float

def get_terrain_grid(local_grid_proto):
    """Generate a 3xN set of points representing the terrain local grid."""
    cells_pz_full = unpack_grid(local_grid_proto).astype(np.float32)
    # Populate the x,y values with a complete combination of all possible pairs for the dimensions in the grid extent.
    ys, xs = np.mgrid[0:local_grid_proto.local_grid.extent.num_cells_x,
                    0:local_grid_proto.local_grid.extent.num_cells_y]
    # Numpy vstack makes it so that each column is (x,y,z) for a single terrain point. The height values (z) come from the
    # terrain grid's data field.
    pts = np.vstack(
        [np.ravel(xs).astype(np.float32),
        np.ravel(ys).astype(np.float32), cells_pz_full]).T
    pts[:, [0, 1]] *= (local_grid_proto.local_grid.extent.cell_size,
                    local_grid_proto.local_grid.extent.cell_size)
    return pts

class LocalGridPub(Node):
    def __init__(self):
        super().__init__('local_grid_publisher')
        sdk = bosdyn.client.create_standard_sdk('SpotViz')
        self.robot_ = sdk.create_robot("192.168.80.3")
        bosdyn.client.util.authenticate(self.robot_)
        self.pub_ = self.create_publisher(PointCloud2, 'local_grid', 1)
        self.timer_ = self.create_timer(0.1, self.publish)

    def create_pointcloud(self, tuples, frame_id):
        #Construct a ROS2 PointCloud2 message using a numpy array of tuples
        buf = []
        for pt in tuples:
            buf += struct.pack('fff', *pt)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        pc2 = PointCloud2(
            header= Header(frame_id=frame_id, stamp=self.get_clock().now().to_msg()),
            height=1,
            width=len(tuples),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=12,  # Each point consists of three float32s, each float32 is 4 bytes
            row_step=12 * len(tuples),
            data=bytearray(buf)
        )

        return pc2
        
    def publish(self):
        local_grid_client = self.robot_.ensure_client(LocalGridClient.default_service_name)
        robot_state_client = self.robot_.ensure_client(RobotStateClient.default_service_name)
        proto = local_grid_client.get_local_grids(['terrain'])


        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == 'terrain':
                vision_tform_local_grid = get_a_tform_b(
                                local_grid_found.local_grid.transforms_snapshot, VISION_FRAME_NAME,
                                local_grid_found.local_grid.frame_name_local_grid_data).to_proto()
                cell_size = local_grid_found.local_grid.extent.cell_size
                cell_count = local_grid_found.local_grid.extent.num_cells_x * local_grid_found.local_grid.extent.num_cells_y
                terrain_pts = get_terrain_grid(local_grid_found)

        x_base = vision_tform_local_grid.position.x + cell_size * 0.5
        y_base = vision_tform_local_grid.position.y + cell_size * 0.5



        robot_state = robot_state_client.get_robot_state()
        vision_tform_ground_plane = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                              VISION_FRAME_NAME, BODY_FRAME_NAME)
        
        z_ground_in_vision_frame = vision_tform_ground_plane.position.z
        z = np.ones(cell_count, dtype=np.float32)
        z *= z_ground_in_vision_frame

        
        terrain_pts[:, 0] += x_base
        terrain_pts[:, 1] += y_base

        terrain_pts[:, 2] -= z

        #Filter out z values close to 0
        mask = np.abs(terrain_pts[:, 2]) >= 0.03 + z
        terrain_pts = terrain_pts[mask]


        pc2 = self.create_pointcloud(terrain_pts, "body")
        self.pub_.publish(pc2)

        return True


def main() -> None:
    rclpy.init()
    local_grid_publisher = LocalGridPub()
    rclpy.spin(local_grid_publisher)

if __name__ == "__main__":
    main()
