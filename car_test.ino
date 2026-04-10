#include <SimpleKalmanFilter.h>
#include <Adafruit_LSM6DSO32.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>

// Objects
Adafruit_LSM6DSO32 IMU;
File dataFile;
Servo myservo;
String dataBuffer;

SimpleKalmanFilter GYRO_FILTER(10.47, 10.47, 10);

// Pins
const int chipSelect = 53;
const int red_led = 2;
const int blue_led = 4;
const int green_led = 3;
const int buzzer = 5;

// Timing
unsigned long currentMillis = 0;
unsigned long dtPrevMillis = 0;
float dt;

// Calibration
const int NUM_OF_SAMPLES = 1000;
int COUNTER;

// IMU variables
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

float GYRO_X, GYRO_Y, GYRO_Z;
float OFFSET_X, OFFSET_Y, OFFSET_Z;
float GYRO_AVG_X, GYRO_AVG_Y, GYRO_AVG_Z;

// Control variables
float Kp = 5.9e-4, Ki = 20e-5;  // UNCHANGED
float setpoint = 0;
float error;
float integral = 0;
float integral_limit = 1000;    // ✅ added limit

float n; // RPMk,

// const float max_angle = 0.0872665; // 5 deg
float AoA = 0;

// Servo mapping
int MIN_SIGNAL = 600;
float MIN_DEGREE = -15*PI/180; // min angle in rads
int MAX_SIGNAL = 720;
float MAX_DEGREE = 15*PI/180; // max angle in rads

int servo_pos;

// Safety
// float MAX_GYRO_RANGE = 5;

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(buzzer, OUTPUT);

  Wire.begin();

  START_UP();
  IMU_SETP();
  SD_SETUP();
  CALIBRATE_AND_OFFSET();

  myservo.attach(6);

  servo_pos = fmap(0, MIN_DEGREE, MAX_DEGREE, MIN_SIGNAL, MAX_SIGNAL);
  myservo.writeMicroseconds(servo_pos);

  dataBuffer.reserve(1024);

  dtPrevMillis = millis();
}

// ================= LOOP =================
void loop() {
  currentMillis = millis();

  // ===== SLOW LOOP so that code runs once every 20 ms=====
  static unsigned long lastControl = 0;
  if (currentMillis - lastControl < 20) return;
  lastControl = currentMillis;

  dt = (currentMillis - dtPrevMillis) / 1000.0;
  dtPrevMillis = currentMillis;

  // Read IMU
  IMU.getEvent(&accel, &gyro, &temp);

  // Store IMU data into variables
  GYRO_X = gyro.gyro.x + OFFSET_X;
  GYRO_Y = gyro.gyro.y + OFFSET_Y;
  GYRO_Z = gyro.gyro.z + OFFSET_Z;
  ACCEL_X = accel.acceleration.x;
  ACCEL_Y = accel.acceleration.y;
  ACCEL_Z = accel.acceleration.z;

  // Convert to RPM
  n = GYRO_X * (60 / (2 * PI));

  // Filter RPM data
  filtered_n = GYRO_FILTER.updateEstimate(GYRO_X);

  // ===== PI CONTROL =====
  error = setpoint - filtered_n;
  if (abs(error) < 1.0) {  // 1 RPM deadband (tune this)
    error = 0;
  }

  // Calculate PI values
  integral += error * dt;
  // integral = max(-integral_limit, min(integral_limit, integral));   // Integral windup protection
  AoA = Kp * error + Ki * integral;

  // Clamp AoA to always stay within min and max angles used in fmap
  AoA = max(MIN_DEGREE, min(MAX_DEGREE, AoA));

  // Safety check
  // if (abs(GYRO_Y) >= MAX_GYRO_RANGE || abs(GYRO_Z) >= MAX_GYRO_RANGE) {
  //   AoA = 0;
  // }

  // Servo command
  servo_pos = fmap(AoA, MIN_DEGREE, MAX_DEGREE, MIN_SIGNAL, MAX_SIGNAL);
  myservo.writeMicroseconds(servo_pos);

  // ===== SERIAL LOGGING =====
  Serial.print("RPM: ");
  Serial.print(n);
  Serial.print(" | AoA: ");
  Serial.print(AoA);
  Serial.print(" | Servo: ");
  Serial.println(servo_pos);

  // ===== SD LOGGING =====
  dataBuffer += String(ACCEL_X, 6); dataBuffer += " ";
  dataBuffer += String(ACCEL_Y, 6); dataBuffer += " ";
  dataBuffer += String(ACCEL_Z, 6); dataBuffer += " ";
  dataBuffer += String(GYRO_X, 6); dataBuffer += " ";
  dataBuffer += String(GYRO_Y, 6); dataBuffer += " ";
  dataBuffer += String(GYRO_Z, 6); dataBuffer += " ";
  dataBuffer += String(n, 6);      dataBuffer += " ";
  dataBuffer += String(filtered_n, 6); dataBuffer += " ";
  dataBuffer += String(AoA, 6);    dataBuffer += " ";
  dataBuffer += String(currentMillis, 4); dataBuffer += " ";
  dataBuffer += String(servo_pos); dataBuffer += "\r\n";

  unsigned int chunkSize = dataFile.availableForWrite();
  if (chunkSize && dataBuffer.length() >= chunkSize) {
    dataFile.write(dataBuffer.c_str(), chunkSize);
    dataFile.flush();
    dataBuffer.remove(0, chunkSize);
  }
}

// ================= FUNCTIONS =================

// Calibration
void CALIBRATE_AND_OFFSET() {
  Serial.println("Starting calibration...");

  for (COUNTER = 0; COUNTER < NUM_OF_SAMPLES;) {
    if (IMU.gyroscopeAvailable()) {
      COUNTER++;

      IMU.getEvent(&accel, &gyro, &temp);

      GYRO_X = gyro.gyro.x;
      GYRO_Y = gyro.gyro.y;
      GYRO_Z = gyro.gyro.z;

      GYRO_AVG_X += GYRO_X;
      GYRO_AVG_Y += GYRO_Y;
      GYRO_AVG_Z += GYRO_Z;
    }
  }

  OFFSET_X = -(GYRO_AVG_X / COUNTER);
  OFFSET_Y = -(GYRO_AVG_Y / COUNTER);
  OFFSET_Z = -(GYRO_AVG_Z / COUNTER);

  Serial.println("Calibration complete!");
}

// Startup checks
void START_UP() {
  if (!SD.begin(chipSelect)) {
    while (1) {
      setColor(255, 0, 0, 1);
    }
  }

  if (!IMU.begin_I2C()) {
    while (1) {
      setColor(255, 0, 0, 1);
    }
  }
}

// SD setup
void SD_SETUP() {
  String fileName = "TEST1.txt";

  int count = 1;
  while (SD.exists(fileName)) {
    fileName = "TEST" + String(count++) + ".txt";
  }

  dataFile = SD.open(fileName, FILE_WRITE);
  dataFile.println("GX GY GZ RPM AoA Servo");
  dataFile.flush();

  Serial.print("Logging to: ");
  Serial.println(fileName);
}

// IMU config
void IMU_SETP() {
  IMU.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  IMU.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
  IMU.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  IMU.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
}

// Mapping
int fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return int((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min;
}

// LED
void setColor(int red, int green, int blue, bool toggle) {
  if (toggle) {
    analogWrite(red_led, red);
    analogWrite(green_led, green);
    analogWrite(blue_led, blue);
  } else {
    analogWrite(red_led, 0);
    analogWrite(green_led, 0);
    analogWrite(blue_led, 0);
  }
}