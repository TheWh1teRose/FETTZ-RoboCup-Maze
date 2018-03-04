#include <BnrOneA.h>   // Bot'n Roll ONE A library
#include <EEPROM.h>    // EEPROM reading and writing
#include <SPI.h>       // SPI communication library required by BnrOne.cpp
#include <Wire.h>
BnrOneA one; 

#define SSPIN  2       // Slave Select (SS) pin for SPI communication
#define ADDRESS 0x60                                      // Defines address of CMPS10


void setup() {
  // put your setup code here, to run once:
  Wire.begin();                                               // Conects I2C
  Serial.begin(57600);
  one.spiConnect(SSPIN);   // start SPI communication module
  one.stop();              // stop motors

  directions();
}

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



void loop() {
  // put your main code here, to run repeatedly:
delay (1500);
turnright();
}




int TurnArray [4] = {0,0,0,0};
void directions(){
  for (int i=0; i<4; i++){
    while (true){
        if (one.readButton() == 1){
          TurnArray[i] = read_bearing();
          one.lcd1(TurnArray[0], TurnArray[1], TurnArray[2], TurnArray[3]);
          break;
          }
      }
      delay (250);
    }
  one.lcd1(TurnArray[0], TurnArray[1], TurnArray[2], TurnArray[3]);
}



int count = 1;
void turnright(){
  int nowBearing = read_bearing();
  int finishBearing = TurnArray[count];
  one.lcd2(nowBearing, finishBearing);
if (nowBearing < finishBearing){
    while (nowBearing < finishBearing){
    nowBearing = read_bearing();
    one.move(16,-16);
    one.lcd2(nowBearing, finishBearing,1);
    }
  }
else{
while(nowBearing > 200){
  nowBearing = read_bearing();
  one.move(16,-16);
  one.lcd2(nowBearing, finishBearing,2.1);
  }
while (nowBearing < finishBearing){
  nowBearing = read_bearing();
  one.move(16,-16);
  one.lcd2(nowBearing, finishBearing,2.2);
  }
    }
  one.stop();
  count = count + 1;
  if (count == 4){
    count = 0;
    }
}








 












