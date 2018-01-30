#include <BnrOneA.h>   // Bot'n Roll ONE A library
#include <EEPROM.h>    // EEPROM reading and writing
#include <SPI.h>       // SPI communication library required by BnrOne.cpp
#include <BnrRescue.h>
#include <Wire.h>
BnrOneA one;
BnrRescue brm;

#define MODULE_ADDRESS 0x2C
#define SSPIN  2
#define ADDRESS 0x60
#define  Measure  1     //Mode select
int URECHO = 3;         // PWM Output 0-25000US,Every 50US represent 1cm
int URTRIG = 5;         // PWM trigger pin
int sensorPin = A0;     // select the input pin for the potentiometer
int sensorValue = 0;    // variable to store the value coming from the sensor

byte sonarL=0, sonarC=0, sonarR=0;
byte rgbL[3]={0,0,0};
byte rgbR[3]={0,0,0};
int  lenghtSquare = 32;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);     // set baud rate to 57600bps for printing values at serial monitor.
  one.spiConnect(SSPIN);   // start SPI communication module
  brm.i2cConnect(MODULE_ADDRESS);   //Enable I2C communication
  brm.setModuleAddress(0x2C);       //Change I2C Address
  brm.setSonarStatus(ENABLE);       //Enable/Disable Sonar scanning
  brm.setRgbStatus(ENABLE);
  one.stop();

  pinMode(URTRIG,OUTPUT);                    // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                 // Set to HIGH
  pinMode(URECHO, INPUT);

  //calibrateCMPS11();
  while(one.readButton()!=1){
    delay(50);
  }

}

int phase = 1; //1 ist discovery mode, 2 ist logical search

void loop() {
  //erste tests damit sich der Roboter immer rechts hält
  if(phase==1){
    //Auslesen der Ultraschallsensoren (nowC ist der Mittlere)
    byte nowC=0;
    brm.readSonars(&sonarL,&nowC,&sonarR);
    //checken ob rechts eine Wand ist
    if(getSideDist()>20){
      one.move(20,-20);
      delay(600);
      one.stop();
      moveOneSquare();
    }else{
      //checken ob vor dem ROboter eine Wand liegt
      if(nowC<10){
        one.move(20,-20);
        delay(600);
        one.stop();
      }else{
        moveOneSquare();
      }
    }
  }
  delay(100);
}


//Ein Feld nach vorne bewegen
void moveOneSquare(){
  //Sensoren einlesen
  byte nowC=0;
  brm.readSonars(&sonarL,&nowC,&sonarR);
  int now = (int)nowC;
  int finish = now - 30; //Feldgröße = 30 cm
  one.move(20,20);
  while(true){
    //aktualisieren der Sensoren
    byte nowC=0;
    brm.readSonars(&sonarL,&nowC,&sonarR);
    int now = (int)nowC;
    if(now<=finish || now<=7){
      break;
    }
  }
  //one.stop();
}

//Halbes Feld nach vorne bewegen
void moveHalfSquare(){
  //Sensoren einlesen
  byte nowC=0;
  brm.readSonars(&sonarL,&nowC,&sonarR);
  int now = (int)nowC;
  int finish = now - 16;
  one.move(20,20);
  while(true){
    byte nowC=0;
    brm.readSonars(&sonarL,&nowC,&sonarR);
    one.lcd1((int)sonarL, (int)nowC, (int)sonarR);
    int now = (int)nowC;
    if(now<=finish || now<=7){
      break;
    }
  }
  one.stop();
}

//drehen des Roboters mit hilfe des Gyroskops
void turn(int degree){
  float now = read_bearing();
  float finish = now + degree;
  if(finish >= 360){
    finish = finish - 360;
  }
  bool overZero = false;
  while(true){

    now = read_bearing();
    one.lcd1((int)now, (int)finish);
    one.lcd2(overZero);
    if(now>0&&now<(360-degree)){
        overZero=true;
      }
    if((now+degree)>360 && !overZero){
      one.move(6,-6);
    }else if(now < (finish-1)){
      one.move(6,-6);
    }else{
      one.stop();
      break;
    }
  }
}

//gibt true zurück wenn der Roboter einen schwarzen Untergrund sieht
bool backInBlack()
{
  //Einlesen der RGB Sensoren
  byte rgbL[3]={0,0,0};
  byte rgbR[3]={0,0,0};
  brm.readRgbL(&rgbL[0],&rgbL[1],&rgbL[2]);
  brm.readRgbR(&rgbR[0],&rgbR[1],&rgbR[2]);

  //one.lcd1((int)rgbR[0], (int)rgbR[1], (int)rgbR[2]);
  if((int)rgbR[0] < 150){
    return true;
  }else{
    return false;
  }
}

//gibt true zurück wenn der Robter auf einen Silbernen Untergrund steht
bool onSilver()
{
  //Einlesen der RGB Sensoren
  byte rgbL[3]={0,0,0};
  byte rgbR[3]={0,0,0};
  brm.readRgbL(&rgbL[0],&rgbL[1],&rgbL[2]);
  brm.readRgbR(&rgbR[0],&rgbR[1],&rgbR[2]);

  //one.lcd1((int)rgbR[0], (int)rgbR[1], (int)rgbR[2]);
  if((int)rgbR[0] < 253){
    return true;
  }else{
    return false;
  }
}

//Helpercode zum auslesen des Gyroscops
float read_bearing()
{
byte highByte, lowByte;    // highByte and lowByte store the bearing and fine stores decimal place of bearing

   Wire.beginTransmission(ADDRESS);           //start communication with CMPS10
   Wire.write(2);                             //Send the register we wish to start reading from
   Wire.endTransmission();

   Wire.requestFrom(ADDRESS, 2);              // Request 4 bytes from CMPS10
   while(Wire.available() < 2);               // Wait for bytes to become available
   highByte = Wire.read();
   lowByte = Wire.read();

return (float)((highByte<<8)+lowByte)/10;
}

//Helpercode zum Calibrieren den Gyroscops
void calibrateCMPS11()
{
   one.move(-30,30); // Slowly rotate the compass on the horizontal plane in all directions

   Wire.beginTransmission(ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF0);                          //Calibration sequence byte 1
   Wire.endTransmission();
   delay(30);

   Wire.beginTransmission(ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF5);                          //Calibration sequence byte 2
   Wire.endTransmission();
   delay(30);

   Wire.beginTransmission(ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF7);                          //Calibration sequence byte 2
   Wire.endTransmission();
   delay(30);

   one.move(-20,20); // Slowly rotate the compass on the horizontal plane in all directions
   delay(15000);

   Wire.beginTransmission(ADDRESS);           //start communication with CMPS10
   Wire.write(0);                             //Send the register we wish to start reading from
   Wire.write(0xF8);                          //Exit calibration mode
   Wire.endTransmission();
   delay(30);
   one.move(0,0); // Stop rotation

}

//Gibt die Distance zu der Rechten Seite des ROboters zurück
int getSideDist()                              // a low pull on pin COMP/TRIG  triggering a sensor reading
{
  Serial.print("Distance Measured=");
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses

  unsigned int DistanceMeasured= 0;
  if( Measure)
  {
    unsigned long LowLevelTime = pulseIn(URECHO, LOW) ;
    if(LowLevelTime>=45000)                 // the reading is invalid.
    {
      Serial.print("Invalid");
    }
    else{
    DistanceMeasured = LowLevelTime /50;   // every 50us low level stands for 1cm
    Serial.print(DistanceMeasured);
    Serial.println("cm");
  }
  }
  else {
    sensorValue = analogRead(sensorPin);
    if(sensorValue<=10)                   // the reading is invalid.
    {
      Serial.print("Invalid");
    }
    else {
    sensorValue = sensorValue*0.718;
    Serial.print(sensorValue);
    Serial.println("cm");
    }
  }
  return DistanceMeasured;
}
