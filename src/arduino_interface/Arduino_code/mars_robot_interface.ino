// Write this script onto an Arduino UNO with a L293 motor shield. It is expected that motors 1 and
// 2 are the right motors, 3 and 4 the left ones. The script waits for messages on the serial 
// connection in a custom format to read out the commanded speeds for the corresponding motors. Then
// it sets the speeds accordingly, using the AFMotor.h library for Arduino. You might need to 
// install this library with your Arduino IDE first to be able to compile the code. Search for servo
// motor library

#include "AFMotor.h"

// Identifiers for the motors
const int MOTOR_1 = 1; 
const int MOTOR_2 = 2; 
const int MOTOR_3 = 3; 
const int MOTOR_4 = 4; 

// Create motor objects with 64KHz pwm
AF_DCMotor motor1(MOTOR_1, MOTOR12_64KHZ);
AF_DCMotor motor2(MOTOR_2, MOTOR12_64KHZ);
AF_DCMotor motor3(MOTOR_3, MOTOR12_64KHZ);
AF_DCMotor motor4(MOTOR_4, MOTOR12_64KHZ);

// Create arrays to store the current speeds
const byte numChars = 4; // 3 digits for the number (0-255) and 1 digit to mark end of number
char speedLeft[numChars];
char speedRight[numChars];

// Flag whether we detected new messages
boolean newData = false;

// Setup method, gets executed once on startup
void setup() {
  // Timeout for the serial connection
  Serial.setTimeout(1);
  // Begin the serial connection with a baud rate of 9600
  Serial.begin(9600);
  // Wait for the serial connection to build up
  while (Serial.available() <= 0) {
    delay(100);
  }
  // Serial.println("<Arduino is ready>");
}

// The loop function that loops as long as the Arduino is powered
void loop() {
  // Receive speed msg over serial connection (if the is any)
  receiveMsg();
  // If new speed msg was received, print is back over serial connection for debugging purposes
  showNewData();
  // Set motor velocites according to most recent speed message
  motor_velocity();
}

void receiveMsg() {
  // Current index in speed array
  static byte ndx = 0;
  
  // If a msg is available on serial connection (FIFO) (else loop over and do nothing)
  if (Serial.available() > 0) {
    // Read current char
    char rc = Serial.read();

    // Detect for 'A', meaning that a speed for the left wheel follows. If char doesn't match, skip 
    // and enter next loop iteration
    if (rc == 'A') {
      // As long as we do not detect 'B', the read chars are part of speed left
      while (rc != 'B') {
        // If a msg is available on serial connection (FIFO) (else loop over and do nothing)
        if (Serial.available() > 0) {
          // Read current char
          rc = Serial.read();
          // Check for 'B', this would mean that speed left is fully received
          if (rc != 'B') {
              // The char was not 'B', so it is part of speed left. Save it in the array for speed 
              // left
              speedLeft[ndx] = rc;
              // Increase index where to write the array next
              ndx++;
              // For safety: Limit index to array length
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          // If char is 'B', we know that speed left is fully detected. Therefore, end speed left 
          // string by adding '\0' and reset the write-index to 0
          } else {
              speedLeft[ndx] = '\0';
              ndx = 0;
          }
        }
      }
      // Detect speed right. We know that all chars until '\n' belong to this speed
      while (rc != '\n') {
        // If a msg is available on serial connection (FIFO) (else loop over and do nothing)
        if(Serial.available() > 0) {
          // Read current char
          rc = Serial.read();
          // As long as char is not the terminating char
          if (rc != '\n') {
            // Save char in array for speed right
            speedRight[ndx] = rc;
            // Increase index where to write array next
            ndx++;
            // For safety: Limit index to array length
            if (ndx >= numChars) {
                ndx = numChars - 1;
          }
          // If char is '\n', we know that speed right is fully detected. Therefore, end speed right 
          // string by adding '\0' and reset the write-index to 0
          } else {
              speedRight[ndx] = '\0';
              ndx = 0;
              // We have received a new speed for left and right that we want to update the motors 
              // with. Mark this by setting the newData flag to true
              newData = true;
          }
        }
      }
    }
  }
}

// This method sends the received speed back over the serial connection for debugging purposes
void showNewData() {
  // Only run if new speeds were received
  if (newData == true) {
    // Print speeds over serial
    Serial.print(speedLeft);
    Serial.print(" and ");
    Serial.println(speedRight);
    // Set newData flag to false to only print this message once
    newData = false;
  } 
}

// This method sends the most recent speeds to the motors. This gets executed each iteration of 
// loop(), also if the values didn't actually change
void motor_velocity() {
  // Convert speed char arrays into strings and then to Integers  
  String speed_left_string = String(speedLeft);
  int speed_left = speed_left_string.toInt();
  String speed_right_string = String(speedRight); 
  int speed_right = speed_right_string.toInt();
  
  // Right motors
  // Switch case depending on the speed value
  if (speed_right >= 255) {
    motor1.setSpeed(255);
    motor2.setSpeed(255);
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  else if (speed_right < 255 && speed_right > 0) {
    motor1.setSpeed(speed_right);
    motor2.setSpeed(speed_right);
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  else if (speed_right < 0 && abs(speed_right) >= 255) {
    motor1.setSpeed(255);
    motor2.setSpeed(255);
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  else if (speed_right < 0 && abs(speed_right) < 255) {
    motor1.setSpeed(abs(speed_right));
    motor2.setSpeed(abs(speed_right));
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  else if (speed_right == 0) {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }
  // Catch strange behavior
  else {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }
  
  // Left motors
  // Switch case depending on the speed value
  if (speed_left >= 255) {
    motor3.setSpeed(255);
    motor4.setSpeed(255);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  else if (speed_left < 255 && speed_left > 0) {
    motor3.setSpeed(speed_left);
    motor4.setSpeed(speed_left);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  else if (speed_left < 0 && abs(speed_left) >= 255) {
    motor3.setSpeed(255);
    motor4.setSpeed(255);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  else if (speed_left < 0 && abs(speed_left) < 255) {
    motor3.setSpeed(abs(speed_left));
    motor4.setSpeed(abs(speed_left));
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  else if (speed_left == 0) {
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  }
  // Catch strange behavior
  else {
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    }
}