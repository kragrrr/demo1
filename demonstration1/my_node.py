import rclpy
from rclpy.node import Node
import time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class my_node(Node):
    def __init__(self):
        super().__init__('demo1_node')
        qos_policy = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.started = False
        self.exploring = False
        self.returning = False
        self.markers_seen = False
        self.start_attempts = 0
        self.start_time = time.monotonic()

        self.explore_path = Path()
        self.explore_path.header.frame_id = 'odom'
        self.return_path = Path()
        self.return_path.header.frame_id = 'odom'

        self.pub_cmd = self.create_publisher(String, '/cmd', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/ecte477/slam_map', qos_policy)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_explore_path = self.create_publisher(Path, '/ecte477/explorer', 10)
        self.pub_return_path = self.create_publisher(Path, '/ecte477/home', 10)

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.callback_map, qos_policy)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
        self.sub_stack = self.create_subscription(MarkerArray, '/stack_points', self.callback_stack_points, 10)

        self.start_timer = self.create_timer(1.0, self.check_start_time)
        self.get_logger().info('demo1_node started! Will send start command in 15 seconds...')

    def check_start_time(self):
        elapsed = time.monotonic() - self.start_time
        if elapsed >= 15.0 and not self.started:
            msg = String()
            msg.data = 'start'
            self.pub_cmd.publish(msg)
            self.start_attempts += 1
            self.get_logger().info(f'Start command sent (attempt {self.start_attempts})...')
            if self.start_attempts >= 5:
                self.started = True
                self.exploring = True
                self.start_timer.cancel()
                self.get_logger().info('Exploration beginning!')

    def callback_map(self, data):
        self.pub_map.publish(data)

    def callback_odom(self, data):
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose

        if self.exploring:
            self.explore_path.poses.append(pose)
            self.pub_explore_path.publish(self.explore_path)
        elif self.returning:
            self.return_path.poses.append(pose)
            self.pub_return_path.publish(self.return_path)

    def callback_stack_points(self, stack_points):
        if self.exploring:
            if len(stack_points.markers) > 0:
                self.markers_seen = True
            elif self.markers_seen and len(stack_points.markers) == 0:
                self.exploring = False
                self.get_logger().info('Exploration complete, waiting 8 seconds before returning home')
                self.return_timer = self.create_timer(8.0, self.return_home)

    def return_home(self):
        self.return_timer.cancel()
        self.returning = True
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 0.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.pub_goal.publish(goal_msg)
        self.get_logger().info('Returning home to (0, 0, 0)')

def main(args=None):
    rclpy.init(args=args)
    mn = my_node()
    rclpy.spin(mn)

if __name__ == '__main__':
    main()
