# Arduino Interface

This is a package to send velocity command to an Arduino over USB serial connection. It consists of 
node 'arduino_interface'. This node listens to a topic 'dynamic_joint_states' that contains the 
speed information for the left and right wheels. The speed is mapped on a integer range from 0 
(= 0 m/s) to 255 (= 1 m/s). The speed information is then passed over the serial connection to the
Arduino. The passing happens as a string message with a custom created format. 


To start the node, run 
```
ros2 run arduino_interface arduino_interface
```
#### On the Arduino
To use the Arduino interface, you need to **upload corresponding code to the Arduino**. We provide 
suitable code for an Arduino UNO that has servo motors connected via a L293 motor shield 
(`Arduino_code/mars_robot_interface.ino`) or via a L298 motor driver 
(`Arduino_code/L298.ino`). Upload one of these using the Arduino IDE. You might need to install the 
servo motor library in you Arduino IDE first.