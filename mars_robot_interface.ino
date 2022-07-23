#include "AFMotor.h"

const int MOTOR_1 = 1; 
const int MOTOR_2 = 2; 
const int MOTOR_3 = 3; 
const int MOTOR_4 = 4; 

AF_DCMotor motor1(MOTOR_1, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor2(MOTOR_2, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor3(MOTOR_3, MOTOR12_64KHZ); // create motor object, 64KHz pwm
AF_DCMotor motor4(MOTOR_4, MOTOR12_64KHZ); // create motor object, 64KHz pwm

const byte numChars = 5;
char receivedChars1[numChars];   // an array to store the received data
char receivedChars2[numChars];

boolean newData = false;

void setup() {
  Serial.setTimeout(1);
  Serial.begin(9600);
  while (Serial.available() <= 0) {
    delay(100);
  }
  // Serial.println("<Arduino is ready>");
}

void loop() {
  recvWithEndMarker();
  showNewData();
  motor_velocity();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  
  if (Serial.available() > 0) {
    char rc = Serial.read();
  
    if (rc == 'A'){
      // Ersten Teil detektieren
      while (rc != 'B') {
        if(Serial.available() > 0){
          rc = Serial.read();
  
          if (rc != 'B') {
              receivedChars1[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          } else {
              receivedChars1[ndx] = '\0'; // terminate the string
              ndx = 0;
          }
        }
      }
      // Zweiten Teil detektieren
      while (rc != '\n') {
        if(Serial.available() > 0){
          rc = Serial.read();
  
          if (rc != '\n') {
              receivedChars2[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          } else {
              receivedChars2[ndx] = '\0'; // terminate the string
              ndx = 0;
              newData = true;
          }
        }
      }
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print(receivedChars1);
    Serial.print(" and ");
    Serial.println(receivedChars2);
    newData = false;
  } 
}

void motor_velocity() {
  while (Serial.available()==0){} // wait for input
  
  String speed_left_string = String(receivedChars1);
  int speed_left = speed_left_string.toInt();
  String speed_right_string = String(receivedChars2); 
  int speed_right = speed_right_string.toInt();
  
  // RIGHT MOTOR
  if (speed_right >= 255){
    //MOTOR_B CLOCKWISE MAX SPEED
    motor1.setSpeed(255);
    motor2.setSpeed(255);
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  else if (speed_right < 255 && speed_right> 0){  
    //MOTOR_B CLOCKWISE MAX SPEED
    motor1.setSpeed(speed_right);
    motor2.setSpeed(speed_right);
    motor1.run(FORWARD);
    motor2.run(FORWARD);
  }
  else if (speed_right < 0 && abs(speed_right)>= 255){
    //MOTOR_B COUNTERCLOCKWISE MAX SPEED
    motor1.setSpeed(255);
    motor2.setSpeed(255);
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  else if (speed_right < 0 && abs(speed_right)< 255){
    motor1.setSpeed(abs(speed_right));
    motor2.setSpeed(abs(speed_right));
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
  }
  else if (speed_right == 0){
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
  }
  else {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    }
  
  
// LEFT MOTOR
//MOTOR_A CLOCKWISE MAX SPEED
  if (speed_left >= 255){
    //MOTOR_B CLOCKWISE MAX SPEED
    motor3.setSpeed(255);
    motor4.setSpeed(255);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  else if (speed_left < 255 && speed_left> 0){  
    //MOTOR_B CLOCKWISE MAX SPEED
    motor3.setSpeed(speed_left);
    motor4.setSpeed(speed_left);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
  else if (speed_left < 0 && abs(speed_left)>= 255){
    //MOTOR_B COUNTERCLOCKWISE MAX SPEED
    motor3.setSpeed(255);
    motor4.setSpeed(255);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  else if (speed_left < 0 && abs(speed_left)< 255){
    motor3.setSpeed(abs(speed_left));
    motor4.setSpeed(abs(speed_left));
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
  }
  else if (speed_left == 0){
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
  }
  else {
    motor3.setSpeed(0);
    motor4.setSpeed(0);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    }
}
