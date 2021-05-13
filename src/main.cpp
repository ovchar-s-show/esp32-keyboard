#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "gyroscope.cpp"
#include "BleKeyboard.h"

#define TCAADDR 0x70                      // address of tca9548a, gyro address is in library
#define GYRO_NUMBER 5
// For positions array:
#define POS_UP 0                          // Array index of when angle goes down
#define POS_PRESS 1                       // Array index of when angle remains +- equal
#define GO_UD_COEF 1.5f                   // Angle diff to raise GoesDown or GoesUp if more
#define EQ_UD_COEF 0.5f                   // Angle diff to raise EqualDown or EqualUp if less
#define PRESS_DIFF 16.2f                  // Positions Up and Press diff to press the button

// #define CALIBRATION                    // offsets calculation
// #define DEBUG                          // view the calibration info in console

void tcaSelect(uint8_t i);                // MUX (gyroscope selection)
bool checkVirtualButtonPress(uint8_t i);
void setOffsets();
void press(uint8_t i);
void sendSymbol();
#ifdef CALIBRATION
void calibration();                       // write calcuated gyro offsets to EEPROM
#endif

/*
MAIN COMPONENTS
*/
  Gyroscope gyros[GYRO_NUMBER] = {
    Gyroscope(0.00827f),
    Gyroscope(0.00547f),
    Gyroscope(0.00766f),
    Gyroscope(0.00907f),
    Gyroscope(0.00607f)
  };
  BleKeyboard Keyb = BleKeyboard("MPBoard", "ovchars2");
/*
MAIN COMPONENTS
*/

uint8_t tca = 0;                                                                // MUX selector
float positions[GYRO_NUMBER][2];                                                // pos_up and _press angles for each gyro
float lastPos[GYRO_NUMBER];
float avgDifFX, avgDiffY;
enum States {GoesDown, EqualDown, GoesUp, EqualUp};
States states[GYRO_NUMBER] = {EqualUp, EqualUp, EqualUp, EqualUp, EqualUp};
uint32_t timeTick;
uint8_t pressVal[GYRO_NUMBER] = {0,0,0,0,0};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Keyb.begin();

  for(tca = 0; tca < GYRO_NUMBER; ++tca){     // gyro and lastPos initialization
    tcaSelect(tca);
    gyros[tca].begin();
    lastPos[tca] = gyros[tca].getAngle(tca);
  }
  #ifdef CALIBRATION
    calibration();
  #endif
  setOffsets();
  timeTick = millis();
}

void loop() {
  for(tca = 0; tca < GYRO_NUMBER; ++tca){
    tcaSelect(tca);
    float real = gyros[tca].getAngle(tca);

    if(states[tca] == EqualUp){
      if(lastPos[tca] > real + GO_UD_COEF){
        positions[tca][POS_UP] = real;
        states[tca] = GoesDown;
      }
    }
    else if(states[tca] == GoesDown){
      if(abs(lastPos[tca] - real < EQ_UD_COEF)){
        positions[tca][POS_PRESS] = real;
        states[tca] = EqualDown;
      }
    }
    else if (states[tca] == EqualDown){
      if(lastPos[tca] < real - GO_UD_COEF) {
        states[tca] = GoesUp;
        if(checkVirtualButtonPress(tca)) {
          timeTick = millis();
          press(tca);
        }
      }  
    }
    else if (states[tca] == GoesUp){
      if(abs(lastPos[tca] - real) < EQ_UD_COEF){
        states[tca] = EqualUp;
      }
    }
    lastPos[tca] = real;
  }
  if(millis() - timeTick > 800){
    timeTick = millis();
    sendSymbol();
  }
}

void tcaSelect(uint8_t i) {                  //TCA6548A switching to (0-GYRO_NUMBER)
    if (i > GYRO_NUMBER) return; 
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
}

void calibration(){
    #ifdef DEBUG
    Serial.println("\nCalculating offsets for gyros.");
    #endif
    for(tca = 0; tca < GYRO_NUMBER; ++tca){
        tcaSelect(tca);
        #ifdef DEBUG
        Serial.print("\nGYROSCOPE ");
        Serial.println(tca);
        gyros[tca].calcGyroOffsets(true,0,0);
        #else
        gyros[tca].calcGyroOffsets(false,0,0);
        #endif
    }
    #ifdef DEBUG
    Serial.println("\n\nDone. Writing to EEPROM...");
    #endif
    int index = 0;
    for(tca = 0; tca < GYRO_NUMBER; ++tca){
        EEPROM.put(index, gyros[tca].getGyroXoffset());
        index += sizeof(float);
        EEPROM.put(index, gyros[tca].getGyroYoffset());
        index += sizeof(float);
        EEPROM.put(index, gyros[tca].getGyroZoffset());
        index += sizeof(float);
    }
    #ifdef DEBUG
    Serial.println("\n\nDone. Reading...");
    #endif
    }

void setOffsets(){
    int index = 0;
    float x,y,z;
    for(tca = 0; tca < GYRO_NUMBER; ++tca){
        #ifdef DEBUG 
          Serial.print("\n\nGyro ");
          Serial.print(tca);
          Serial.print("...");
        #endif
        EEPROM.get(index, x);
        index += sizeof(float);
        EEPROM.get(index, z);
        index += sizeof(float);
        EEPROM.get(index, z);
        index += sizeof(float);
        gyros[tca].setGyroOffsets(x,y,z);
        #ifdef DEBUG
          Serial.print("\b\b\b Read: ");
          Serial.print(gyros[tca].getGyroXoffset());
          Serial.print(";\t");
          Serial.print(gyros[tca].getGyroYoffset());
          Serial.print(";\t");
          Serial.print(gyros[tca].getGyroZoffset());
          Serial.println(".");
        #endif
    }
    #ifdef DEBUG
        Serial.println("Check the information, start in 5 seconds");
        delay(5000);
    #endif
}

bool checkVirtualButtonPress(uint8_t i){
    return positions[i][POS_UP] > positions[i][POS_PRESS] + PRESS_DIFF;
}

void press(uint8_t i){
    for(uint8_t a = 0; a < GYRO_NUMBER; ++a){
        if (a == i) ++a;
        if(pressVal[a] > 0) sendSymbol();
    }
    if(pressVal[i] < 7) pressVal[i] += 1;
}

void sendSymbol(){
    char toSend = 0;
    if(pressVal[0] > 0){
        if(pressVal[0] == 1) toSend = 'w';
        else if(pressVal[0] == 2) toSend = 'l'; 
        else if(pressVal[0] == 3) toSend = ',';
        else if(pressVal[0] == 4) toSend = 'i';
        else if(pressVal[0] == 5) toSend = 'p';
        else if(pressVal[0] == 6) toSend = 'v';
        pressVal[0] = 0;
    }
    if(pressVal[1] > 0){
        if(pressVal[1] == 1) toSend = 'o';
        else if(pressVal[1] == 2) toSend = 'r';
        else if(pressVal[1] == 3) toSend = 'b';
        else if(pressVal[1] == 4) toSend = 'j';
        else if(pressVal[1] == 5) toSend = 'q';
        else if(pressVal[1] == 6) toSend = 'x';
        pressVal[1] = 0;
    }
    if(pressVal[2] > 0){
        if(pressVal[2] == 1) toSend = 'd';
        else if(pressVal[2] == 2) toSend = 'h';
        else if(pressVal[2] == 3) toSend = 'c';
        else if(pressVal[2] == 4) toSend = 'k';
        else if(pressVal[2] == 5) toSend = 's';
        else if(pressVal[2] == 6) toSend = 'y';
        pressVal[2] = 0;
    }
    if(pressVal[3] > 0){
        if(pressVal[3] == 1) toSend = 'e';
        else if(pressVal[3] == 2) toSend = '\b';
        else if(pressVal[3] == 3) toSend = 'f';
        else if(pressVal[3] == 4) toSend = 'm';
        else if(pressVal[3] == 5) toSend = 't';
        else if(pressVal[3] == 6) toSend = 'z';
        pressVal[3] = 0;
    }
    if(pressVal[4] > 0){
        if(pressVal[4] == 1) toSend = ' ';
        else if(pressVal[4] == 2) toSend = 'a';
        else if(pressVal[4] == 3) toSend = 'g';
        else if(pressVal[4] == 4) toSend = 'n';
        else if(pressVal[4] == 5) toSend = 'u';
        else if(pressVal[4] == 6) toSend = '!';
        pressVal[4] = 0;
    }
    if(toSend != 0) {
        Serial.print(toSend);
        Keyb.print(toSend);
    }
}