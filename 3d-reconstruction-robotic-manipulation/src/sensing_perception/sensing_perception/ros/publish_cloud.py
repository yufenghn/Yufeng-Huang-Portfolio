import os
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def read_ascii_ply_xyzrgb(path: str):
    """Read an ASCII PLY written by our SfM node: x y z r g b per line."""
    with open(path, 'r') as f:
        if not f.readline().startswith("ply"):
            raise RuntimeError("Not a PLY file")
        if "ascii" not in f.readline():
            raise RuntimeError("Only ASCII PLY supported")
        line = f.readline()
        while not line.startswith("element vertex"):
            line = f.readline()
        n_points = int(line.split()[-1])
        # skip until end_header
        while "end_header" not in line:
            line = f.readline()

        xs, ys, zs, rs, gs, bs = [], [], [], [], [], []
        for _ in range(n_points):
            vals = f.readline().strip().split()
            if len(vals) < 6:
                continue
            x, y, z = map(float, vals[:3])
            r, g, b = map(float, vals[3:6])
            xs.append(x); ys.append(y); zs.append(z)
            rs.append(r); gs.append(g); bs.append(b)

    xyz = np.vstack([xs, ys, zs]).T.astype(np.float32)
    rgb = np.vstack([rs, gs, bs]).T.astype(np.uint8)
    return xyz, rgb


class CloudPublisher(Node):
    def __init__(self):
        super().__init__('cloud_publisher')
        self.declare_parameter('ply_path', '')
        self.declare_parameter('frame_id', 'map')

        self.ply_path = self.get_parameter('ply_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        if not self.ply_path or not os.path.exists(self.ply_path):
            self.get_logger().fatal(f"PLY file not found: {self.ply_path}")
            raise SystemExit(1)

        self.get_logger().info(f"Loading point cloud from {self.ply_path}")
        self.xyz, self.rgb = read_ascii_ply_xyzrgb(self.ply_path)

        self.pub = self.create_publisher(PointCloud2, 'object_cloud', 1)
        self.timer = self.create_timer(0.5, self.publish_cloud)
        self.get_logger().info(f"Loaded {self.xyz.shape[0]} points — publishing on /object_cloud")

    def publish_cloud(self):
        # Build xyz+rgb tuples for sensor_msgs_py
        points = []
        for (x, y, z), (r, g, b) in zip(self.xyz, self.rgb):
            rgb_uint32 = struct.unpack('I', struct.pack('BBBB', int(b), int(g), int(r), 255))[0]
            points.append((float(x), float(y), float(z), rgb_uint32))

        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32,  count=1),
        ]

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        msg = point_cloud2.create_cloud(header, fields, points)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
