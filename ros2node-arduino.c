int incomedate = 0;
void setup() {
  Serial.begin(9600); 
}

void loop() {
  if (Serial.available())
  {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);     
    //Serial.print("ok i get:"); 
    Serial.write(Serial.read());
  }
}
