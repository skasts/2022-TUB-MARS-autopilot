// Example 2 - Receive with an end-marker

const byte numChars = 5;
char receivedChars1[numChars];   // an array to store the received data
char receivedChars2[numChars];
int ena = 5;
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int enb = 10;

boolean newData = false;

void setup() {
  Serial.setTimeout(1);
  Serial.begin(115200);
  while (Serial.available() <= 0) {
    delay(100);
  }
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
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
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(enb, 255);
  }
  else if (speed_right < 255 && speed_right> 0){  
    //MOTOR_B CLOCKWISE MAX SPEED
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(enb, speed_right);
  }
  else if (speed_right < 0 && abs(speed_right)>= 255){
    //MOTOR_B COUNTERCLOCKWISE MAX SPEED
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    analogWrite(enb, 255);
  }
  else if (speed_right < 0 && abs(speed_right)< 255){
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);
    analogWrite(enb, abs(speed_right));
  }
  else if (speed_right == 0){
    //STOP
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  }
  else {
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    }
  
  
// LEFT MOTOR
//MOTOR_A CLOCKWISE MAX SPEED
   if (speed_left >= 255){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    analogWrite(ena, 255);
  }
  else if (speed_left < 255 && speed_left> 0){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    analogWrite(ena, speed_left);
  }
  else if (speed_left < 0 && abs(speed_left)>= 255){
    //MOTOR_A COUNTERCLOCKWISE MAX SPEED
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(ena, 255);
  }
  else if (speed_left < 0 && abs(speed_left)< 255){
    //MOTOR_A COUNTERCLOCKWISE MAX SPEED
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    analogWrite(ena, abs(speed_left));
  }
  else if (speed_left == 0){
    //STOP
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  } 
  else {
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }

}
