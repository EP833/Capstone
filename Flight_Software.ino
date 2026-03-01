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



SimpleKalmanFilter GYRO_FILTER(10.47, 10.47, 1);  // Filter for the roll axis


const int chipSelect = 53;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long dtPrevMillis = 0;
unsigned long launchStart = 0;
unsigned long launchTimer = 0;



const int NUM_OF_SAMPLES = 1000;    // Controls how many samples will be taken while gyroscope is being calibrated
const int CALIBRATION_TIMER = 400;  // Number of miliseconds until LED is toggled while being calibrated
const int IDLE_TIMER = 2000;        // Number of miliseconds until LED is toggled while idle
const int LAND_TIMER = 500;         // Number of miliseconds until LED is toggled while rocket has landed
int COUNTER;                        // Used in gyroscope calibration


// Change these to set when rocket states get changed
const long LAUNCH_ACCEL = 35;         // Minimum G force rocket experiences to be considered launched / IN m/s^2
const long HEIGT_ABOVE_PAD = 30;      // Minimum height above pad where rocket is considered launched / IN FT
const long HEIGT_BELOW_APOGEE = 200;  // Minimum height below apogee where rocket is considered falling / IN FT

// Used for audio & visual feedback on rocket
const int buzzer = 5;     //Pin number for buzzer
const int red_led = 2;    //Pin number for red ;ed
const int blue_led = 4;   //Pin number for blue led
const int green_led = 3;  //Pin number for green led


sensors_event_t accel;      // Used to hold IMU data
sensors_event_t gyro;       // Used to hold IMU data
sensors_event_t temp;       // Used to hold IMU data
bool ACCEL_LAUNCH = false;  // Used to check if rocket has launched by checking acceleration
bool ALT_LAUNCH = false;    // Used to check if rocket has launched by checking alitude
bool LED_STATE = true;      // Used to toggle LEDs



// Used to change the different states of the rocket launch
// 1 = rocket idles on launch pad
// 2 = rocket has been launched
// 3 = rocket has passed apogee
int state = 1;

// Setup IMU variables
float GYRO_X, GYRO_Y, GYRO_Z, FILTER_DATA, n;  // Holds gyroscope data in X, Y, Z
float GYRO_AVG_X, GYRO_AVG_Y, GYRO_AVG_Z;      // Used in gyroscope calibration
float OFFSET_X, OFFSET_Y, OFFSET_Z;            // Used in gyroscope calibration
float ACCEL_X, ACCEL_Y, ACCEL_Z, MAG_ACCEL;    // Holds accel data in X, Y, Z and magnitude
float PRESSURE, ALTITUDE, PAD_ALT, APOGEE;     // Holds pressure, altitude, altitude of the pad

// Define control variables
float Kp = 3e-5, Ki = 1e-3;         // Proportional & Integral   Gains
float setpoint = 100;               // Goal value (rpm)
float error;                        // Difference between setpoint and current value
float integral = 0;                 // The accumalitive error in integration
float dt;                           // Sampling Interval
bool reached_target_1 = false;      // Used for checking to see if first spin rate has been reached
bool reached_target_2 = false;      // Used for checking to see if second spin rate has been reached
float spin_time = 0;                // Holds how long rocket has spun for
const float hold_time = 2;          // How long to hold spin (s)
const float max_angle = 0.0872665;  // max angle servos can be rotated (5 degree in rad)
float servo_pos;                    // variable to store the servo position
float AoA;                          // Angle of attack to set servos to (rad)

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  START_UP();                //Starts the IMU, Barometer, and SD Card
  IMU_SETP();                // Sets up the IMU
  SD_SETUP();                // Sets up the SD
  CALIBRATE_AND_OFFSET();    // Calibrates gyroscope
  myservo.attach(6);         // attaches the servo on pin 9 to the servo object
  dataBuffer.reserve(1024);  // reserve 1 kB for String used as a dataBuffer
}

void loop() {
  currentMillis = millis();  // Get how long program has ran for in milliseconds
  switch (state) {
    case 1:
      {
        if (currentMillis - previousMillis > IDLE_TIMER) {  // Toggles LED based on IDLE_TIMER
          digitalWrite(LED_BUILTIN, LED_STATE);
          previousMillis = currentMillis;
          LED_STATE = !LED_STATE;
        }
        IMU.getEvent(&accel, &gyro, &temp);                       // Get all the IMU data and put into variables
        BARO.read();                                              // Read barometer pressure sensore
        GYRO_X = gyro.gyro.x + OFFSET_X;                          // Put gyroscope data into each variable and add the offset
        GYRO_Y = gyro.gyro.y + OFFSET_Y;                          //
        GYRO_Z = gyro.gyro.z + OFFSET_Z;                          //
        ACCEL_X = accel.acceleration.x;                           // Put accelerometer data into each variable
        ACCEL_Y = accel.acceleration.y;                           //
        ACCEL_Z = accel.acceleration.z;                           //
        MAG_ACCEL = calc_mag_accel(ACCEL_X, ACCEL_Y, ACCEL_Z);    // Find magnitude of acceleration
        PRESSURE = BARO.getPressure();                            // Put pressure reading into variable
        ALTITUDE = BARO.getAltitudeFeet();                        // Put altitude reading into variable
        if (MAG_ACCEL > LAUNCH_ACCEL && ACCEL_LAUNCH == false) {  // Checks to see if we have launched by reading acceleromter data
          ACCEL_LAUNCH = true;
        }
        if ((ALTITUDE - PAD_ALT) > HEIGT_ABOVE_PAD && ALT_LAUNCH == false) {  // Checks to see if we have launched by reading altitude data
          ALT_LAUNCH = true;
        }
        if (ACCEL_LAUNCH == true && ALT_LAUNCH == true) {  // If both checks are true then considered rocket launched
          Serial.println("State set to: 2");
          dtPrevMillis = currentMillis;  // Used to calculate dt
          launchStart = currentMillis;   // Saves the current time as the time that rocket was launched
          state = 2;                     // Changes state to 2 so rocket is considered launched
        }
      }
      break;
    case 2:
      {
        dt = (currentMillis - dtPrevMillis) / 1000;          // Calucate dt
        launchTimer = (currentMillis - launchStart) / 1000;  // Calculate to see how long rocket has been considered launched
        IMU.getEvent(&accel, &gyro, &temp);                  // Get all the IMU data and put into variables
        BARO.read();                                         // Read barometer pressure sensore
        GYRO_X = gyro.gyro.x + OFFSET_X;                     // Put gyroscope data into each variable and add the offset
        GYRO_Y = gyro.gyro.y + OFFSET_Y;                     //
        GYRO_Z = gyro.gyro.z + OFFSET_Z;                     //
        n = GYRO_Y * (60 / (2 * PI));                        // Convert the spin of the rocket from rad/s to rpm
        FILTER_DATA = GYRO_FILTER.updateEstimate(GYRO_Y);    // Get the data but filtered **MIGHT BE TAKEN OUT**
        ACCEL_X = accel.acceleration.x;                      // Put accelerometer data into each variable
        ACCEL_Y = accel.acceleration.y;                      //
        ACCEL_Z = accel.acceleration.z;                      //
        PRESSURE = BARO.getPressure();                       // Put pressure reading into variable
        ALTITUDE = BARO.getAltitudeFeet();                   // Put altitude reading into variable
        APOGEE = max(APOGEE, ALTITUDE);                      // Checks to see if current alitude can be considered apogee

        dataBuffer += String(GYRO_X, 8);       // Turn sesor data into string and keep the first 8 decimals
        dataBuffer += " ";                     // Leave a space between each sensor data item
        dataBuffer += String(GYRO_Y, 8);       //
        dataBuffer += " ";                     //
        dataBuffer += String(GYRO_Z, 8);       //
        dataBuffer += " ";                     //
        dataBuffer += String(FILTER_DATA, 8);  //
        dataBuffer += " ";                     //
        dataBuffer += String(ACCEL_X, 4);      //
        dataBuffer += " ";                     //
        dataBuffer += String(ACCEL_Y, 4);      //
        dataBuffer += " ";                     ///
        dataBuffer += String(ACCEL_Z, 4);      //
        dataBuffer += " ";                     //
        dataBuffer += String(PRESSURE, 4);     //
        dataBuffer += " ";                     //
        dataBuffer += String(ALTITUDE, 4);     ///
        dataBuffer += " ";                     //
        dataBuffer += String(currentMillis);   //
        dataBuffer += "\r\n";                  // Start a new line for each new data point
        Serial.print("Unsaved data buffer length (in bytes): ");
        Serial.print(dataBuffer.length());
        Serial.print(" Time: ");
        Serial.println(currentMillis);

        // check if the SD card is available to write data without blocking
        // and if the dataBuffered data is enough for the full chunk size
        unsigned int chunkSize = dataFile.availableForWrite();
        if (chunkSize && dataBuffer.length() >= chunkSize) {
          digitalWrite(LED_BUILTIN, HIGH);                // Turn LED on
          dataFile.write(dataBuffer.c_str(), chunkSize);  // Write to SD card
          digitalWrite(LED_BUILTIN, LOW);                 // Turn LED off
          dataFile.flush();                               //Saves data to SD Card incase we lose power
          dataBuffer.remove(0, chunkSize);                // remove written data from dataBuffer
        }

        /////// START OF FIN CONTROL ////////
        if (launchTimer >= 3 && (reached_target_1 == false || reached_target_2 == false)) {
          error = setpoint - n;                                    // Find the error between the setpoint and the current RPM
          integral = integral + error * dt;                        // Calculates the integral
          AoA = Kp * error + Ki * integral;                        // Find new angle of attack based on PI gains
          AoA = max(-max_angle, min(max_angle, AoA));              // Does not allow the AoA to be greater than 5° or less than -5°
          servo_pos = round(fmap(AoA, -0.005, 0.005, 600, 1000));  // Convert our AoA to a servo command by mapping **NEEDS TO FIXED**
          myservo.writeMicroseconds(servo_pos);                    // Send command to servo

          if ((abs(n) > (abs(setpoint) - 2)) && (abs(n) < (abs(setpoint) + 2))) {  //Checks to see if we are in acceptable range for 100 RPM test
            spin_time += dt;                                                       // If in acceptable range then add how much time has passed to the timer
          }

          if (spin_time >= hold_time) {      // If held for set time swap to next setpoint
            setpoint = -setpoint;            // Swap setpoint to next setpoint (ONLY USED IN 100 RPM TEST)
            if (reached_target_1 == true) {  // If first setpoint flag true then make the second one true
              reached_target_2 = true;       // Set second setpoint flag true
            }                                //
            reached_target_1 = true;         // Set first setpoint flag true
            spin_time = 0;                   // Reset spin time
          }
        } else {
          // SET SERVOS TO ORIGINAL POSITION **NEEDS TO BE FIXED**
        }
        /////// END OF FIN CONTROL ////////

        if ((APOGEE - ALTITUDE) > HEIGT_BELOW_APOGEE) {  // Checks to see if we have passed apogee by an amount
          dataFile.close();                              // Close SD Card to save all data
          Serial.println("State set to: 3");             //
          state = 3;                                     // Change to next state where rocket is considered landed
        }                                                //
        dtPrevMillis = currentMillis;                    // Used to calculate dt
        break;
      }
    case 3:
      {
        if (currentMillis - previousMillis > LAND_TIMER) {  // Toggles LED based on LAND_TIMER
          digitalWrite(LED_BUILTIN, LED_STATE);
          previousMillis = currentMillis;
          LED_STATE = !LED_STATE;
        }
      }
      break;
  }
}

// Calculates the gryoscope offset
void CALIBRATE_AND_OFFSET() {
  Serial.println("Starting calibration...");
  for (COUNTER = 0; COUNTER < NUM_OF_SAMPLES;) {                 // Run test for how many times NUM_OF_SAMPLES is
    if (IMU.gyroscopeAvailable() == 1) {                         // If gyroscope is working run test
      currentMillis = millis();                                  //Get the current time
      COUNTER++;                                                 // Increment counter
      sensors_event_t accel;                                     // Get a new normalized sensor event
      sensors_event_t gyro;                                      //
      sensors_event_t temp;                                      //
      IMU.getEvent(&accel, &gyro, &temp);                        // Put all IMU data into each variable
      GYRO_X = gyro.gyro.x;                                      // PUT GYRO DATA INTO VARIBLES
      GYRO_Y = gyro.gyro.y;                                      //
      GYRO_Z = gyro.gyro.z;                                      //
      GYRO_AVG_X += GYRO_X;                                      // Add up all the gyroscope values to be averaged later
      GYRO_AVG_Y += GYRO_Y;                                      //
      GYRO_AVG_Z += GYRO_Z;                                      //
      if (currentMillis - previousMillis > CALIBRATION_TIMER) {  // Blink LED to show test is running
        digitalWrite(LED_BUILTIN, LED_STATE);
        previousMillis = currentMillis;
        LED_STATE = !LED_STATE;
      }
    }
  }

  OFFSET_X = -(GYRO_AVG_X / COUNTER);  // Calculate offset for each axis
  OFFSET_Y = -(GYRO_AVG_Y / COUNTER);
  OFFSET_Z = -(GYRO_AVG_Z / COUNTER);



  BARO.read();                       // Read barometer pressure sensore
  PAD_ALT = BARO.getAltitudeFeet();  // Get the first height as the height above the pad
  GYRO_AVG_X = 0;
  GYRO_AVG_Y = 0;
  GYRO_AVG_Z = 0;
  Serial.println("Calibration over!");
}

void START_UP() {  // Starts the IMU, Barometer, and SD Card
  // Check to see if SD Card working
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  } else {
    Serial.println("SD Card Found!");
  }


  // Check to see if barometer working
  if (BARO.begin() == true) {
    Serial.print("MS5611 found: ");
    Serial.println(BARO.getAddress());
  } else {
    Serial.println("MS5611 not found. halt.");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }


  // Check to see if IMU working
  if (!IMU.begin_I2C()) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  } else {
    Serial.println("LSM6DSO32 Found!");
  }
  digitalWrite(LED_BUILTIN, LOW);
}


// Sets up the SD Card
void SD_SETUP() {

  String fileName;
  String testName = "T_";
  int testRun = 1;
  String fileType = ".txt";
  fileName = testName;
  fileName += testRun;
  fileName += fileType;  // Combines all the parts to make a file name Ex: T_3.txt


  while (SD.exists(fileName)) {       // Checks to see if the file already exists on the SD card
    digitalWrite(LED_BUILTIN, HIGH);  //Turn LED on
    testRun += 1;                     // Increment testRun so we can test for next file
    fileName = testName;              //
    fileName += testRun;              //
    fileName += fileType;             // Combines all the parts to make a file name Ex: T_3.txt
    digitalWrite(LED_BUILTIN, LOW);   // Turn LED off
  }


  Serial.print("Saving data to '");
  Serial.print(fileName);
  Serial.print("'. Have a safe flight!");
  Serial.println();
  dataFile = SD.open(fileName, FILE_WRITE);
  dataFile.write("Gyro X | Gyro Y | Gyro Z | Gyro Filter | Accel X | Accel Y | Accel Z | Pressure | Altitude | Time\r\n");
  dataFile.flush();  // ^ Write to SD card where each variable will be saved and save data to SD card
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

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return float((x - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min;
}
