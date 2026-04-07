#include <Servo.h>

Servo myServo;


/*
int LOW_SIGNAL = 710
Fin 1: 79.7 degrees
Fin 2: 79.0 degrees
Fin 3: 80.7 degrees
Fin 4: 80.6 degrees
Fin Avg: 80 degrees
*/

/*
int HIGH_SIGNAL = 630
Fin 1: 98.6 degrees
Fin 2: 99.1 degrees
Fin 3: 99.2 degrees
Fin 4: 99.9 degrees
Fin Avg = 99.2 degrees
*/

int LOW_SIGNAL = 710;
float LOW_SIGNAL_ANGLE = -0.1745329252;  // rad
int HIGH_SIGNAL = 630;
float HIGH_SIGNAL_ANGLE = 0.1605702912;  // rad
float fin_angle;
int pos;
char KEEP_SIGNAL[10];


int state = 1;


String inputString = "";
bool inputComplete = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(6);  // servo signal pin
  inputString.reserve(10);

  Serial.println(fmap(-0.0872664626, LOW_SIGNAL_ANGLE, HIGH_SIGNAL_ANGLE, LOW_SIGNAL, HIGH_SIGNAL));
}

void loop() {

  if (inputComplete) {
    pos = inputString.toInt();
    myServo.writeMicroseconds(pos);
    Serial.print("Servo Command: ");
    Serial.println(pos);
    // clear input
    inputString = "";
    inputComplete = false;
  }
}

// This runs automatically when serial data arrives
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == '\n') {
      inputComplete = true;
    }
  }
}

int fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return int((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min;
}
