import rclpy

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

# See nav2/tester_node.py for inspiration

class CovarianceAdder(Node):
# get velocity from dynamic joint states
    def __init__(self):
        super().__init__('covariance_adder')

        # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        self.declare_parameter('pose_covariance_diagonal', [0.01, 0.01, 0.01, 0.01, 0.01, 0.1])
        self.pose_covariance_diagonal = self.get_parameter('pose_covariance_diagonal')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            'orb_slam2_mono_node/pose',
            self.callback,
            qos_profile=qos_profile)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'slam/pose',
            10
        )

    def callback(self, msg):
        # self.info_msg(str(msg.pose.position.x))
        # self.info_msg(str(new_msg.pose.covariance[7]))
        new_msg = PoseWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.pose.pose = msg.pose
        for i in range(6):
            new_msg.pose.covariance[i*7] = self.pose_covariance_diagonal.value[i]
        self.pose_pub.publish(new_msg)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)
        # self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

def main(args=None):
    rclpy.init(args=args)
    covariance_adder = CovarianceAdder()
    rclpy.spin(covariance_adder)
    covariance_adder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()