"""Non-blocking ROS observations for the desktop panel."""
import time
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class StatusNode(Node):
    def __init__(self):
        super().__init__('go2_control_panel', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.seen = {}
        self.pose_stamp = None
        self.last_clock = None
        self.subs = [
            self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data),
            self.create_subscription(PointCloud2, '/livox/lidar', lambda _: self.mark('雷达'), qos_profile_sensor_data),
            self.create_subscription(Odometry, '/Odometry', lambda _: self.mark('FAST-LIO'), qos_profile_sensor_data),
            self.create_subscription(PoseWithCovarianceStamped, '/localization/pose', self.pose, 10),
        ]
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.nav = self.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        self.save = self.create_client(Trigger, '/map_save')
        self.nav_future = None
        self.nav_requested = 0
        self.nav_active = False
        self.nav_checked = 0

    def mark(self, name):
        self.seen[name] = time.monotonic()

    def clock(self, msg):
        stamp = (msg.clock.sec, msg.clock.nanosec)
        if stamp != self.last_clock:
            self.mark('仿真时钟')
        self.last_clock = stamp

    def pose(self, msg):
        if msg.header.frame_id == 'map':
            self.pose_stamp = msg.header.stamp
            self.mark('定位')

    def fresh_stamp(self, stamp):
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
        return self.get_clock().now().nanoseconds > 0 and -.2 <= age <= 2

    def values(self):
        now = time.monotonic()
        states = {k: now - self.seen.get(k, -100) < 5 for k in ['仿真时钟', '雷达', 'FAST-LIO', '定位']}
        states['定位'] &= self.pose_stamp is not None and self.fresh_stamp(self.pose_stamp)
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
            states['TF'] = states['仿真时钟'] and self.fresh_stamp(tf.header.stamp)
        except Exception:
            states['TF'] = False
        if self.nav_future is not None and self.nav_future.done():
            try:
                self.nav_active = self.nav_future.result().success
            except Exception:
                self.nav_active = False
            self.nav_checked = now
            self.nav_future = None
        if self.nav_future is not None and now - self.nav_requested > 4:
            self.nav.remove_pending_request(self.nav_future)
            self.nav_future = None
            self.nav_active = False
        if self.nav_future is None and now - self.nav_requested >= 2 and self.nav.service_is_ready():
            self.nav_future = self.nav.call_async(Trigger.Request())
            self.nav_requested = now
        states['Nav2'] = self.nav_active and now - self.nav_checked < 6
        return states
