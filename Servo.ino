#include <Servo.h>

Servo myservo;  // create servo object to control a servo

// twelve servo objects can be created on most boards

int pos = 0;  // variable to store the servo position

void setup() {
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
}

void loop() {

    myservo.writeMicroseconds(pos);

  for (pos =600; pos <= 1200; pos += 10) {
    Serial.println(pos);
    myservo.writeMicroseconds(pos);
    delay(25);
  }
  for (pos = 1200; pos >= 600; pos -= 10) {
    Serial.println(pos);
    myservo.writeMicroseconds(pos);
    delay(25);
  }
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return float((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min;
}
