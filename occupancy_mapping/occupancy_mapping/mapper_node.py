"""Build an occupancy grid from /scan, using TF to find where the laser was.

Subscribes: scan (sensor_msgs/LaserScan)
Uses TF:    map_frame -> scan.header.frame_id
Publishes:  map (nav_msgs/OccupancyGrid, transient local, like map_server)
Services:   save_map (std_srvs/Trigger) -> writes <map_dir>/<map_name>.pgm + .yaml
            clear_map (std_srvs/Trigger)
"""
import math
import os

import rclpy
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from occupancy_mapping.grid import OccupancyGrid


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Mapper(Node):

    def __init__(self):
        super().__init__('mapper')
        d = self.declare_parameter
        self.map_frame = d('map_frame', 'odom').value
        self.map_dir = os.path.expanduser(d('map_dir', '~/maps').value)
        self.map_name = d('map_name', 'map').value
        self.every_n = max(1, d('process_every_n_scans', 2).value)
        self.max_use_range = d('max_use_range', 5.0).value
        self.params = dict(
            width_m=d('width_m', 12.0).value,
            height_m=d('height_m', 12.0).value,
            resolution=d('resolution', 0.05).value,
            origin=(d('origin_x', -1.0).value, d('origin_y', -1.0).value),
            p_hit=d('p_hit', 0.7).value,
            p_miss=d('p_miss', 0.4).value,
        )
        self.grid = OccupancyGrid(**self.params)
        self.scan_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGridMsg, 'map', map_qos)
        self.create_service(Trigger, 'save_map', self.on_save)
        self.create_service(Trigger, 'clear_map', self.on_clear)
        self.create_timer(d('publish_period', 1.0).value, self.publish_map)
        self.get_logger().info(
            f'Mapping in frame "{self.map_frame}": {self.grid.width}x{self.grid.height} cells '
            f'@ {self.grid.resolution} m')

    def on_scan(self, scan):
        self.scan_count += 1
        if self.scan_count % self.every_n:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, scan.header.frame_id, Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().warn(f'No transform yet: {ex}', throttle_duration_sec=2.0)
            return
        t = tf.transform
        self.grid.integrate_scan(t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation),
                                 scan.ranges, scan.angle_min, scan.angle_increment,
                                 scan.range_min, scan.range_max, self.max_use_range)

    def publish_map(self):
        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(self.grid.resolution)
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = float(self.grid.origin[0])
        msg.info.origin.position.y = float(self.grid.origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.to_ros_data()
        self.map_pub.publish(msg)

    def on_save(self, request, response):
        try:
            os.makedirs(self.map_dir, exist_ok=True)
            base = os.path.join(self.map_dir, self.map_name)
            self.grid.to_pgm(base + '.pgm')
            with open(base + '.yaml', 'w') as f:
                f.write(f'image: {self.map_name}.pgm\n'
                        f'mode: trinary\n'
                        f'resolution: {self.grid.resolution}\n'
                        f'origin: [{self.grid.origin[0]}, {self.grid.origin[1]}, 0.0]\n'
                        f'negate: 0\n'
                        f'occupied_thresh: 0.65\n'
                        f'free_thresh: 0.25\n')
            response.success = True
            response.message = f'Saved {base}.pgm and {base}.yaml'
        except OSError as ex:
            response.success = False
            response.message = f'Could not save map: {ex}'
        self.get_logger().info(response.message)
        return response

    def on_clear(self, request, response):
        self.grid = OccupancyGrid(**self.params)
        response.success = True
        response.message = 'Map cleared'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
