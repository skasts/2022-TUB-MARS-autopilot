import rclpy

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class CovarianceAdder(Node):
    # Constructor
    def __init__(self):
        # Call super constructor and give this node a name
        super().__init__('covariance_adder')

        # Declare node parameters: The pose covariance of the slam pose. We also set a default 
        # value, to not force the user to set this. The positional arguments of the double array
        # are:
        # |x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis]
        self.declare_parameter('pose_covariance_diagonal', [0.01, 0.01, 0.01, 0.01, 0.01, 0.1])
        self.pose_covariance_diagonal = self.get_parameter('pose_covariance_diagonal')

        # Set a quality of service (QoS) profile for the subsription of the pose topic. This is set
        # to be 'best effort' with a depth of 1, so that we don't get lagging messages. It is more
        # important to get the recent messages instead of forcing ROS to no lose and message
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Create the subsrciption of the pose topic. The pose msgs on this topic are expectet to be
        # of type PoseStamped without covariance information
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'orb_slam2_mono_node/pose',
            self.callback,
            qos_profile=qos_profile)

        # Create publisher that publishes the received pose msgs enriched with the covariance
        # information on a new topic
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'slam/pose',
            10
        )

    def callback(self, msg):
        # Debug: Publish parts of the received and about to publish msgs
        # self.info_msg(str(msg.pose.position.x))
        # self.info_msg(str(new_msg.pose.covariance[7]))

        # Create an empty msg of type PoseWithCovarianceStamped
        new_msg = PoseWithCovarianceStamped()
        
        # Copy header from received msg
        new_msg.header = msg.header

        # Cope pose from received msg
        new_msg.pose.pose = msg.pose

        # Add covariance information
        for i in range(6):
            new_msg.pose.covariance[i*7] = self.pose_covariance_diagonal.value[i]
        
        # Publish msg
        self.pose_pub.publish(new_msg)

    # Some nice formatted terminal output methods ...
    def info_msg(self, msg: str):
        self.get_logger().info(msg)
        # self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    # Create covariance adder node
    covariance_adder = CovarianceAdder()
    # Spin the node until shutdown
    rclpy.spin(covariance_adder)
    # After shutdown command received, kill the node
    covariance_adder.destroy_node()
    # Shut down rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()