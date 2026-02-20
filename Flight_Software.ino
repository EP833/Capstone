#include <SimpleKalmanFilter.h>
#include <Adafruit_LSM6DSO32.h>
#include <SPI.h>
#include <SD.h>
#include <MS5611.h>
#include <Servo.h>


// Create instances for IMU, SD, BARO
Adafruit_LSM6DSO32 IMU;
File dataFile;
MS5611 BARO(0x77);
Servo myservo;      // create servo object to control a servo
String dataBuffer;  // string to buffer output




const int chipSelect = 53;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

const int FREQUENCY = 1666;
const long HZ = 1 / FREQUENCY;
const int NUM_OF_SAMPLES = 1000;    // Controls how many samples will be taken while gyroscope is being calibrated
const int CALIBRATION_TIMER = 100;  // Controls how fast the RGB LED will blink gyroscope is being calibrated
const int IDLE_TIMER = 2000;        // Controls how fast the RGB LED will blink while rocket idles on pad
const int LAUNCH_TIMER = 1000;      // Controls how fast data is written into dataBuffer (Higher number means writing data more slowly)
const int APOGEE_TIMER = 2000;      // Controls how fast the RGB LED will blink while rocket has landed
int COUNTER;
int pos = 700;  // variable to store the servo position



// Change these to set when rocket states get changed
const long HEIGT_ABOVE_PAD = 40;      // Minimum height above pad where rocket is considered launched / IN FT
const long HEIGT_BELOW_APOGEE = 100;  // Minimum height below apogee where rocket is considered falling / IN FT
const long LAUNCH_ACCEL = 35;         // Minimum G force rocket experiences to be considered launched / IN m/s^2

// Used for audio & visual feedback on rocket
const int buzzer = 5;     //Pin number for buzzer
const int red_led = 2;    //Pin number for red ;ed
const int blue_led = 4;   //Pin number for blue led
const int green_led = 3;  //Pin number for green led


sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;
bool ACCEL_LAUNCH = false;  // Used to check if rocket has launched by checking acceleration
bool ALT_LAUNCH = false;    // Used to check if rocket has launched by checking alitude
bool LED_STATE = true;      // Used to toggle LEDs



// Used to change the different states of the rocket launch
// 1 = rocket idles on launch pad
// 2 = rocket has been launched
// 3 = rocket has passed apogee
int state = 2;

// Setup IMU variables
float GYRO_X, GYRO_Y, GYRO_Z;                // Holds gyroscope data in X, Y, Z
float GYRO_AVG_X, GYRO_AVG_Y, GYRO_AVG_Z;    // Used in gyroscope calibration
float OFFSET_X, OFFSET_Y, OFFSET_Z;          // Used in gyroscope calibration
float ACCEL_X, ACCEL_Y, ACCEL_Z, MAG_ACCEL;  // Holds accel data in X, Y, Z and magnitude
float PRESSURE, ALTITUDE, PAD_ALT, APOGEE;   // Holds pressure, altitude, altitude of the pad



void setup() {
  Serial.begin(9600);
  Wire.begin();
  START_UP();
  IMU_SETP();
  SD_SETUP();
  CALIBRATE_AND_OFFSET();

  // reserve 1 kB for String used as a dataBuffer
  dataBuffer.reserve(1024);
  myservo.attach(6, 700, 1000);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  currentMillis = millis();
  switch (state) {
    case 1:
      {
        if (currentMillis - previousMillis > IDLE_TIMER) {
          digitalWrite(blue_led, LED_STATE);
          tone(buzzer, 1000, 25);
          previousMillis = currentMillis;
          LED_STATE = !LED_STATE;
        }
        IMU.getEvent(&accel, &gyro, &temp);
        BARO.read();  //  note no error checking => "optimistic".
        GYRO_X = gyro.gyro.x + OFFSET_X;
        GYRO_Y = gyro.gyro.y + OFFSET_Y;
        GYRO_Z = gyro.gyro.z + OFFSET_Z;
        ACCEL_X = accel.acceleration.x;
        ACCEL_Y = accel.acceleration.y;
        ACCEL_Z = accel.acceleration.z;
        MAG_ACCEL = calc_mag_accel(ACCEL_X, ACCEL_Y, ACCEL_Z);
        PRESSURE = BARO.getPressure();
        ALTITUDE = BARO.getAltitudeFeet();

        if (MAG_ACCEL > LAUNCH_ACCEL && ACCEL_LAUNCH == false) {
          // Serial.println("MAG SET TRUE");
          ACCEL_LAUNCH = true;
        }
        if ((ALTITUDE - PAD_ALT) > HEIGT_ABOVE_PAD && ALT_LAUNCH == false) {
          // Serial.println("ALT SET TRUE");
          ALT_LAUNCH = true;
        }

        if (ACCEL_LAUNCH == true && ALT_LAUNCH == true) {
          // Serial.println("STATE SET: 2");
          state = 2;
        }
      }
      break;
    case 2:
      {
        IMU.getEvent(&accel, &gyro, &temp);
        BARO.read();  //  note no error checking => "optimistic".

        // Get all sensor data into each variable
        GYRO_X = gyro.gyro.x + OFFSET_X;
        GYRO_Y = gyro.gyro.y + OFFSET_Y;
        GYRO_Z = gyro.gyro.z + OFFSET_Z;
        ACCEL_X = accel.acceleration.x;
        ACCEL_Y = accel.acceleration.y;
        ACCEL_Z = accel.acceleration.z;
        PRESSURE = BARO.getPressure();
        ALTITUDE = BARO.getAltitudeFeet();
        APOGEE = max(APOGEE, ALTITUDE);

        // Add all sensor data to dataBuffer string so that it can be saved to micro sd
        // if (currentMillis - previousMillis > LAUNCH_TIMER) {
        dataBuffer += String(GYRO_X, 4);  // Turn sesor data into string and keep the first 4 decimals
        dataBuffer += "  ";               // Leave a space between each sensor data item
        dataBuffer += String(GYRO_Y, 4);
        dataBuffer += "  ";
        dataBuffer += String(GYRO_Z, 4);
        dataBuffer += "  ";
        dataBuffer += String(ACCEL_X, 4);
        dataBuffer += "  ";
        dataBuffer += String(ACCEL_Y, 4);
        dataBuffer += "  ";
        dataBuffer += String(ACCEL_Z, 4);
        dataBuffer += "  ";
        dataBuffer += String(PRESSURE, 4);
        dataBuffer += "  ";
        dataBuffer += String(ALTITUDE, 4);
        dataBuffer += "\r\n";  // Start a new line for each new data point
        previousMillis = currentMillis;
        Serial.print("Unsaved data buffer length (in bytes): ");
        Serial.println(dataBuffer.length());
        // }

        // check if the SD card is available to write data without blocking
        // and if the dataBuffered data is enough for the full chunk size
        unsigned int chunkSize = dataFile.availableForWrite();
        Serial.print(" Chunk Size: ");
        Serial.println(chunkSize);
        if (chunkSize && dataBuffer.length() >= chunkSize) {
          // write to file and blink LED
          digitalWrite(LED_BUILTIN, HIGH);
          dataFile.write(dataBuffer.c_str(), chunkSize);
          digitalWrite(LED_BUILTIN, LOW);
          dataFile.flush();
          // remove written data from dataBuffer
          dataBuffer.remove(0, chunkSize);
        }

        if (APOGEE - ALTITUDE > HEIGT_BELOW_APOGEE) {
          // Serial.println("STATE SET: 3");
          state = 3;
        }
        break;
      }
    case 3:
      {
        if (currentMillis - previousMillis > APOGEE_TIMER) {
          digitalWrite(green_led, LED_STATE);
          previousMillis = currentMillis;
          LED_STATE = !LED_STATE;
          tone(buzzer, 2000, 50);
          myservo.writeMicroseconds(pos);
          if (pos > 1000) {
            pos = 700;
          }
          pos += 100;
        }
      }
      break;
  }
}

// Calculates the gryoscope offset
void CALIBRATE_AND_OFFSET() {
  Serial.println("Starting calibration...");
  delay(500);
  for (COUNTER = 0; COUNTER < NUM_OF_SAMPLES;) {
    if (IMU.gyroscopeAvailable() == 1) {
      currentMillis = millis();
      COUNTER++;
      // Get a new normalized sensor event
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      IMU.getEvent(&accel, &gyro, &temp);

      // PUT GYRO DATA INTO VARIBLES
      GYRO_X = gyro.gyro.x;
      GYRO_Y = gyro.gyro.y;
      GYRO_Z = gyro.gyro.z;

      GYRO_AVG_X += GYRO_X;
      GYRO_AVG_Y += GYRO_Y;
      GYRO_AVG_Z += GYRO_Z;
      if (currentMillis - previousMillis > CALIBRATION_TIMER) {
        digitalWrite(blue_led, LED_STATE);
        previousMillis = currentMillis;
        LED_STATE = !LED_STATE;
      }
    }
  }

  OFFSET_X = -(GYRO_AVG_X / COUNTER);
  OFFSET_Y = -(GYRO_AVG_Y / COUNTER);
  OFFSET_Z = -(GYRO_AVG_Z / COUNTER);



  BARO.read();                       //  note no error checking => "optimistic".
  PAD_ALT = BARO.getAltitudeFeet();  // Get the first height as the height above the pad
  // Serial.print("Pad Height: ");
  // Serial.println(PAD_ALT, 4);
  GYRO_AVG_X = 0;
  GYRO_AVG_Y = 0;
  GYRO_AVG_Z = 0;
  Serial.println("Calibration over!");
}

// Starts the IMU, Barometer, and SD Card
void START_UP() {
  // Check to see if SD Card working
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    digitalWrite(red_led, HIGH);
    while (true)
      ;
  } else {
    Serial.println("SD Card Found!");
  }


  // Check to see if barometer working
  if (BARO.begin() == true) {
    Serial.print("MS5611 found: ");
    digitalWrite(red_led, HIGH);

    Serial.println(BARO.getAddress());
  } else {
    Serial.println("MS5611 not found. halt.");
    while (1) {
      delay(10);
    }
  }


  // Check to see if IMU working
  if (!IMU.begin_I2C()) {
    while (1) {
      digitalWrite(red_led, HIGH);
      delay(10);
    }
  } else {
    Serial.println("LSM6DSO32 Found!");
  }
  digitalWrite(red_led, LOW);
}


// Sets up the SD Card
void SD_SETUP() {

  String fileName;
  String testName = "T_";
  int testRun = 1;
  String fileType = ".txt";

  fileName = testName;
  fileName += testRun;
  fileName += fileType;


  while (SD.exists(fileName)) {
    Serial.print("File '");
    Serial.print(fileName);
    Serial.print("' already exists. Trying next file.");
    Serial.println();
    testRun += 1;
    fileName = testName;
    fileName += testRun;
    fileName += fileType;
  }


  Serial.print("Saving data to '");
  Serial.print(fileName);
  Serial.print("'. Have a safe flight!");
  Serial.println();
  dataFile = SD.open(fileName, FILE_WRITE);
}


// Sets up the IMU
void IMU_SETP() {
  // Change IMU ranges
  IMU.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  IMU.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);


  // Change IMU data rates
  IMU.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  IMU.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
}

float calc_mag_accel(float x, float y, float z) {
  float mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  return mag;
}