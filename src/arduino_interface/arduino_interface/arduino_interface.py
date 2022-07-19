from codecs import utf_8_decode, utf_8_encode
from encodings import utf_8
import rclpy
import serial
import time
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Header
from rclpy.node import Node
from control_msgs.msg import InterfaceValue
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

Port="/dev/ttyACM1"
baudRate=115200
ser=serial.Serial(Port,baudRate,timeout=1)
msg = DynamicJointState()

class MinimalSubscriber(Node):
# get velocity from dynamic joint states
    def __init__(self):
        super().__init__('minimal_subscriber')
        ser.flush()
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            DynamicJointState,
            'dynamic_joint_states',
            self.listener_callback,
            qos_profile=qos_profile)

    def listener_callback(self, msg):
        speed_left = msg.interface_values[0].values[1] 
        speed_right = msg.interface_values[1].values[1]

        # recalculate speeds for drive-direct
        speed_left = int(int(speed_left*100)*255/1855)
        speed_right = int(int(speed_right*100)*255/1855)

        print(speed_right)
        msg1 = ("A" + str(speed_left) + "B" + str(speed_right) +"\n").encode()
        # msg2 = ("B" + str(speed_right) + "\n").encode()
        ser.write(msg1)
        # ser.write(msg2)
        receive = ser.readline().decode()
        print(receive)
        # print(speed_left, " ", speed_right)
        # time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()