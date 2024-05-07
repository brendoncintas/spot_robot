#!/usr/bin/env python3

from rclpy.action import ActionServer
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from utilities.tf_listener_wrapper import TFListenerWrapper

import spot_driver.conversions as conv
from spot_msgs.action import RobotCommand  # type: ignore



import rclpy
from rclpy.node import Node
import numpy as np
from utilities.simple_spot_commander import SimpleSpotCommander
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.api import local_grid_pb2
import bosdyn.client

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
        #self.publisher_ = self.create_publisher(PoseArray, 'zigzag_marker', 10)
        self.timer_ = self.create_timer(0.5, self.publish)

    
        # Set up basic ROS2 utilities for communicating with the driver
        node = Node("local_grid_publisher")
        name = ""
        namespace = ""
        # tf_listener = TFListenerWrapper(
        #     "local_grid_tf", wait_for_transform=[name + ODOM_FRAME_NAME, name + GRAV_ALIGNED_BODY_FRAME_NAME]
        # )

        robot = SimpleSpotCommander(namespace)

        #robot_command_client = ActionClientWrapper(
        #    RobotCommand, "robot_command", "arm_simple_action_node", namespace=namespace
        #)

        # Claim robot
        node.get_logger().info("Claiming robot")
        result = robot.command("claim")
        if not result.success:
            node.get_logger().error("Unable to claim robot message was " + result.message)
            return False
        node.get_logger().info("Claimed robot")
        
    def publish(self):

        local_grid_client = self.robot_.ensure_client(LocalGridClient.default_service_name)
        proto = local_grid_client.get_local_grids(['terrain'])
        for local_grid_found in proto:
            if local_grid_found.local_grid_type_name == 'terrain':
                terrain_pts = get_terrain_grid(local_grid_found)
        print(terrain_pts)

        # Stand the robot up.
        # node.get_logger().info("Powering robot on")
        # result = robot.command("power_on")
        # if not result.success:
        #     node.get_logger().error("Unable to power on robot message was " + result.message)
        #     return False
        # node.get_logger().info("Standing robot up")
        # result = robot.command("stand")
        # if not result.success:
        #     node.get_logger().error("Robot did not stand message was " + result.message)
        #     return False
        # node.get_logger().info("Successfully stood up.")

        # tf_listener.shutdown()

        return True


def main() -> None:
    rclpy.init()
    local_grid_publisher = LocalGridPub()
    rclpy.spin(local_grid_publisher)

if __name__ == "__main__":
    main()
