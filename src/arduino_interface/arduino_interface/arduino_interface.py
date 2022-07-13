import rclpy
import serial
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Header
from rclpy.node import Node
from control_msgs.msg import InterfaceValue

from std_msgs.msg import String
Port="/dev/ttyUSB5"
baudRate=9600
ser=serial.Serial(Port,baudRate,timeout=1)
msg = DynamicJointState()

# need the topic dynamic joint states, get the velocity, send it as motor commands to the arduino

class MinimalSubscriber(Node):
# get velocity from dynamic joint states
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            DynamicJointState,
            'dynamic_joint_states',
            self.listener_callback, 10)
        

    def listener_callback(self, msg):
        self.get_logger().info('I heard something')
        speed_left = msg.interface_values[0].values[1]
        speed_right =msg.interface_values[1].values[1]     
        # self.get_logger().info("speed_left: %g speed_right: %g" % (speed_left, speed_right))
        # ser.write((str(speed_left) + "," + str(speed_right) + ",").encode())
        str_speed_l = str(speed_left)
        str_speed_r = str(speed_right)
        speeds=[str_speed_l.encode(), str_speed_r.encode()]
        ser.write(10) 
        receive = ser.readline()#get the Data from Arduino
        self.get_logger().info("I got: %s" % (receive))


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()