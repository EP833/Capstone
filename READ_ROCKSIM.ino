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
// 2 = Density
// 3 = yVel
// 4 = Altitude
int excel_pos = 0;


// UNITS
// Time (s)
// yAccel (g)
// Vel (m/s)
// Alt (m)
// Density (kg/m^3)
//

float Prev_Time = 0, Prev_yAccel = 0, Prev_yVel = 0, Prev_Alt = 0, Prev_Density = 0;
float Cur_Time, Cur_yAccel, Cur_yVel, Cur_Alt, Cur_Density;


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
    String list = myFile.readStringUntil('\r');
    int StringSize = list.length();
    for (int i = 1; i < StringSize; i++) {
      // Serial.print(list[i]);
      // Checks to see if next character is a '\t' or a ' '
      if (isSpace(list[i])) {
        buffer[pos] = '\0';
        temp = atof(buffer);
        pos = 0;
        Serial.println(temp, 6);
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
          case 2:
            {
              Cur_Density = temp;
              break;
            }
          case 3:
            {
              Cur_yVel = temp;
              break;
            }
          case 4:
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
      delay(100);
    }
    excel_pos = 0;
    Serial.println(list);
    Serial.print("Time: ");
    Serial.print(Cur_Time);
    Serial.print(" ");
    Serial.print("Accel: ");
    Serial.print(Cur_yAccel);
    Serial.print(" ");
    Serial.print("Density: ");
    Serial.print(Cur_Density);
    Serial.print(" ");
    Serial.print("Vel: ");
    Serial.print(Cur_yVel);
    Serial.print(" ");
    Serial.print("Alt: ");
    Serial.println(Cur_Alt);
    // Finished reading file, stop running program
    while (list.length() == 0) {
      myFile.close();
      while (1)
        ;
    }
  } else {
    Serial.println("ERROR OPENING TEXT");
  }
  //Start of string
  // Read current char
  // If char is a space or a /t then move onto next variable
  // If char is neither then add it to buffer
  // Once next char is a space or /t then make buffer into a float and clear buffer
}