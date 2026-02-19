#include <SPI.h>
#include <SD.h>

File myFile;
// Will hold data value while it is a char array
char buffer[64];
// Will hold data value when char array is converted to float
float temp;
// Will hold current position for buffer
int pos = 0;


// Will be used determine which data value is being recorded
// KEY
// 0 = time
// 1 = yAccel
// 5 = Density
// 6 = yVel
// 7 = Altitude
int excel_pos = 0;


// UNITS
// Time (s)
// yAccel (g)
// Vel (m/s)
// Alt (m)
// Density (kg/m^3)
//

// Define test variables
float Pre_Time = 0, Cur_Time = 0, Cur_yAccel = 0, Cur_yVel = 0, Cur_Alt = 0, Cur_Density = 0;
float spin_time = 0, tolerance = 0.02;
bool reached_target_1 = false;
bool reached_target_2 = false;


// Define constants (Units)
const float y_bar = 0.027244444444444;    // m
const float I_Total = 0.001243311506869;  // kg*m^2
const float A = 0.007006590000000;        // m^2
const float C_Nalpha = 4.690221710596553;
const float hold_time = 2;                     // How long to hold spin (s)
const float C_d = 0.02;                        // Coeficant of damping
const float servo_rate_ideal = 20.944;         // rad/s
const float servo_rate_worst = 0.0174533 * 4;  // rad/s
const float max_angle = 0.0872665;             // 5 degree in rad
const float min_angle = -0.0872665;            // -5 degree in rad


// Define equation variables
float q;             // 1/2 * Density * Velocity ^2
float AoA;           // Angle of attack (rad)
float Prev_AoA = 0;  // Angle of attack (rad)
float Fn = 0;        // q * A * AoA * C_Nalpha
float Moment_A = 0;  // 4 * Fn * y_bar
float Moment_D = 0;  // C_d * Omega
float M_Net;
float Omega = 0;  // Moment_A / I_Total
float Alpha_Omega = 0;
float n = 0;  // Omega / (2*pi)

// Define control variables
float Kp = 3e-5, Ki = 1e-3, Kd = 0;  // Proportional, Integral ,Derivative  Gains
float setpoint = 100;                 // Goal value (rpm)
float error;                          // Difference between setpoint and current value
float prev_error = 0;                 // Difference between setpoint and current value
float integral = 0;                   // The accumalitive error in integration
float derivative = 0;
float dt;           // Sampling Interval
float dAoA;         // Rate of change of the AoA
float AoA_des = 0;  // AoA we want
float maxStep = 0;
float AoA_des_last = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  if (!SD.begin(53)) {
    Serial.println("SD CARD NOT FOUND");
    while (1)
      ;
  }
  Serial.println("SD INIT");

  myFile = SD.open("T_DATA.txt", FILE_READ);
}

void loop() {
  if (myFile) {
    //Start of string
    // Read current char
    // If char is a space or a /t then move onto next variable
    // If char is neither then add it to buffer
    // Once next char is a space or /t then make buffer into a float and clear buffer
    String list = myFile.readStringUntil('\n');
    // Serial.println(list);

    int StringSize = list.length();
    for (int i = 1; i < StringSize; i++) {
      // Checks to see if next character is a '\t' or a ' '
      if (isSpace(list[i]) && i != 1) {
        buffer[pos] = '\0';
        temp = atof(buffer);
        pos = 0;
        switch (excel_pos) {
          case 0:
            {
              Cur_Time = temp;
              break;
            }
          case 1:
            {
              Cur_yAccel = temp;
              break;
            }
          case 5:
            {
              Cur_Density = temp;
              break;
            }
          case 6:
            {
              Cur_yVel = temp;
              break;
            }
          case 7:
            {
              Cur_Alt = temp;
              break;
            }
        }
        excel_pos++;
      } else {
        buffer[pos] = list[i];
        pos++;
      }
    }
    excel_pos = 0;

    // Perform physics equations
    dt = Cur_Time - Pre_Time;
    n = RPM_Eqn(Omega);
    q = Q_Eqn(Cur_Density, Cur_yVel);

    //      ---- CONTROL LOGIC: Reach -> Hold for 2s -> Release ----
    if (Cur_Time >= 2 && (reached_target_1 == false || reached_target_2 == false)) {
      // Find the error between the setpoint and the current RPM
      error = setpoint - n;
      maxStep = dt * servo_rate_worst;
      derivative = (error - prev_error) / dt;
      integral = integral + error * dt;

      AoA_des = Kp * error + Ki * integral + Kd * derivative;
      // AoA = AoA + max(-maxStep, min(maxStep, AoA_des - AoA));  // KEEP
      AoA = AoA_des;                              //
      AoA = max(min_angle, min(max_angle, AoA));  // Does not allow the AoA to be greater than 5° or less than -5°

      if ((abs(n) > (abs(setpoint) - 10)) && (abs(n) < (abs(setpoint) + 10))) {
        spin_time += dt;
      }


      // If held for set time swap to next setpoint
      if (spin_time >= hold_time) {
        setpoint = -setpoint;
        // If first setpoint flag true then make the second one true
        if (reached_target_1 == true) {
          reached_target_2 = true;
        }
        // Set first setpoint flag true
        reached_target_1 = true;
        // Reset spin time
        spin_time = 0;
      }
      prev_error = error;
    } else {
      AoA = 0.00005;
    }

    // Calculate physics equations
    Fn = Fn_Eqn(A, C_Nalpha, q, AoA);
    Moment_A = MoR_Eqn(Fn, y_bar);
    Moment_D = MoD_Eqn(Omega, C_d);
    M_Net = Moment_A - Moment_D;
    Alpha_Omega = Omega_Eqn(M_Net, I_Total);
    Omega = Omega + Alpha_Omega * dt;
    Serial.print("RPM = ");
    Serial.print(n);
    Serial.print(" | ");
    Serial.print("Error = ");
    Serial.print(error, 5);
    Serial.print(" | ");
    Serial.print("AoA = ");
    Serial.print(AoA, 8);
    Serial.print(" | ");
    Serial.print("AoA Des - AoA= ");
    Serial.print(AoA_des - AoA, 8);
    Serial.print(" | ");
    Serial.print("Spin Time = ");
    Serial.print(spin_time, 5);
    Serial.print(" | ");
    Serial.print("Time = ");
    Serial.print(Cur_Time, 4);
    Serial.println();
    // Finished reading file, stop running program
    while (list.length() == 0) {
      myFile.close();
      while (1)
        ;
    }
    Pre_Time = Cur_Time;
  } else {
    Serial.println("ERROR OPENING TEXT");
  }
}

float Omega_Eqn(float MoR, float MoI) {
  float result;
  result = MoR / MoI;
  return result;
}

float RPM_Eqn(float w) {
  float result;
  result = w * (60 / (2 * PI));
  return result;
}

float MoR_Eqn(float Fn, float Y_Bar) {
  float result;
  result = 8 * Fn * Y_Bar;
  return result;
}

float MoD_Eqn(float w, float C_d) {
  float result;
  result = w * C_d;
  return result;
}

float Q_Eqn(float Density, float Velocity) {
  float result;
  result = Velocity * Velocity * Density * 0.5;
  return result;
}

float Fn_Eqn(float Fin_Area, float C, float Q, float Angle_of_Attack) {
  float result;
  result = Fin_Area * C * Q * Angle_of_Attack;
  return result;
}
