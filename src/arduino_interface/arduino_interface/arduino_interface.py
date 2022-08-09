from codecs import utf_8_decode, utf_8_encode
from encodings import utf_8
import rclpy
import serial
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Set the port according to your USB serial connection to Arduino. You can check with 'ls /dev/'.
Port = "/dev/ttyACM0" # <- Set this!!
# Baud rate for the serial connection. 9600 is standard and prooved to be enough
baudRate = 9600
# Create a handle for the serial connection
ser = serial.Serial(Port,baudRate,timeout=1)
# The msg object to temporarily store received messages
msg = DynamicJointState()
 
class ArduinoInterface(Node):
    # Constructor
    def __init__(self):
        # Call super constructor and give this node a name
        super().__init__('arduino_interface')

        # Flush the serial connection in case of old content
        ser.flush()
        
        # Set a quality of service (QoS) profile for the subsription of the dynamic joint state 
        # topic. This is set to be 'best effort' with a depth of 1, so that we don't get lagging
        # messages. It is more important to get the recent messages instead of forcing ROS to no 
        # lose and message
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        # Set the subsription of the 'dynamic joint state' topic. On this topic, the velocities for
        # the wheels are published. As soon as a message on the topic is detected, the 
        # listener_callback gets executed
        self.subscription = self.create_subscription(
            DynamicJointState,
            'dynamic_joint_states',
            self.listener_callback,
            qos_profile=qos_profile)

    # This method gets called every time that we receive a new message on the listened topic
    def listener_callback(self, msg):
        # Get left and right wheel turning velocities from received msg
        speed_left = msg.interface_values[0].values[1] 
        speed_right = msg.interface_values[1].values[1]

        # Map speeds to a range of 0-255 where 1 m/s = 255
        speed_left = int(int(speed_left*100)*255/1855)
        speed_right = int(int(speed_right*100)*255/1855)

        # Wrap speeds in a string where 'A' and 'B' mark the begin of the speed for the left and 
        # right speed. The '\n' marks the end of the message. Send the string over the serial 
        # connection to the Arduino
        msg = ("A" + str(speed_left) + "B" + str(speed_right) +"\n").encode()
        ser.write(msg)
        # For debug purpose, the Arduino sends the received speeds back. We print this to terminal
        receive = ser.readline().decode()
        print(receive)
        
def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    # Create arduino interface node
    arduino_interface = ArduinoInterface()
    # Spin the node until shutdown
    rclpy.spin(arduino_interface)
    # After shutdown command received, kill the node
    arduino_interface.destroy_node()
    # Shut down rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()