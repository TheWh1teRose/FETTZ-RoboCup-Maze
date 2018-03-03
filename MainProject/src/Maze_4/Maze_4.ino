#include <BnrOneA.h>   // Bot'n Roll ONE A library
#include <EEPROM.h>    // EEPROM reading and writing
#include <SPI.h>       // SPI communication library required by BnrOne.cpp
#include <BnrRescue.h>
#include <Wire.h>
#include <math.h>
#include <Ultrasonic.h>
BnrOneA one;
BnrRescue brm;

#define MODULE_ADDRESS 0x2C
#define SSPIN  2
#define ADDRESS 0x60
#define  Measure  1     //Mode select
int URECHO = 3;         // PWM Output 0-25000US,Every 50US represent 1cm
int URTRIG = 5;         // PWM trigger pin
int sensorPin = A0;     // select the input pin for the potentiometer
int URECHO2 = 2;         // PWM Output 0-25000US,Every 50US represent 1cm
int URTRIG2 = 6;         // PWM trigger pin
int sensorPin2 = A1;     // select the input pin for the potentiometer
int sensorValue = 0;    // variable to store the value coming from the sensor

byte sonarL=0, sonarC=0, sonarR=0;
byte rgbL[3]={0,0,0};
byte rgbR[3]={0,0,0};
int  lenghtSquare = 32;

int sensorFArray[] = {0,0,0,0,0};
int sensorHArray[] = {0,0,0,0,0};

#define SUR_TEMP_PIN A0 // Analog input pin connect to temperature sensor SUR pin
#define OBJ_TEMP_PIN A1 // Analog input pin connect to temperature sensor OBJ pin
float temp_calibration=0;       //this parameter was used to calibrate the temperature
//float objt_calibration=0.000; //this parameter was used to calibrate the object temperature
float temperature_range=10;    //we make a map of temperature-voltage according to sensor datasheet. 10 is the temperature step when sensor and 
                               //object distance is 9CM.
float offset_vol=0.014;        //this parameter was used to set the mid level voltage,when put the sensor in normal environment after 10 min,
                               //the sensor output 0.For example,the surrounding temperature is 29℃，but the result is 27℃ via the sensor,
                               //you should set the reerence to 0.520 or more,according to your sensor to change.
                               //the unit is V
float tempValue = 0; 
float objtValue= 0;  
float current_temp=0;
float temp=0;
float temp1=0;
float temp2=0;
unsigned int temp3=0;
const float reference_vol=0.500;
unsigned char clear_num=0;//when use lcd to display
float R=0;
float voltage=0;

long res[100]={
                 318300,302903,288329,274533,261471,249100,237381,226276,215750,205768,
                 196300,187316,178788,170691,163002,155700,148766,142183,135936,130012,
                 124400,119038,113928,109059,104420,100000,95788,91775,87950,84305,
                 80830,77517,74357,71342,68466,65720,63098,60595,58202,55916,
                 53730,51645,49652,47746,45924,44180,42511,40912,39380,37910,
                 36500,35155,33866,32631,31446,30311,29222,28177,27175,26213,
                 25290,24403,23554,22738,21955,21202,20479,19783,19115,18472,
                 17260,16688,16138,15608,15098,14608,14135,13680,13242,12819,
                 12412,12020,11642,11278,10926,10587,10260,9945,9641,9347,
                 9063,8789,8525,8270,8023,7785,7555,7333,7118,6911};
                 
float obj [13][12]={
/*0*/             { 0,-0.274,-0.58,-0.922,-1.301,-1.721,-2.183,-2.691,-3.247,-3.854,-4.516,-5.236}, //
/*1*/             { 0.271,0,-0.303,-0.642,-1.018,-1.434,-1.894,-2.398,-2.951,-3.556,-4.215,-4.931},  //→surrounding temperature,from -10,0,10,...100
/*2*/             { 0.567,0.3,0,-0.335,-0.708,-1.121,-1.577,-2.078,-2.628,-3.229,-3.884,-4.597},   //↓object temperature,from -10,0,10,...110
/*3*/             { 0.891,0.628,0.331,0,-0.369,-0.778,-1.23,-1.728,-2.274,-2.871,-3.523,-4.232},
/*4*/             { 1.244,0.985,0.692,0.365,0,-0.405,-0.853,-1.347,-1.889,-2.482,-3.13,-3.835},
/*5*/             { 1.628,1.372,1.084,0.761,0.401,0,-0.444,-0.933,-1.47,-2.059,-2.702,-3.403},
/*6*/             { 2.043,1.792,1.509,1.191,0.835,0.439,0,-0.484,-1.017,-1.601,-2.24,-2.936},
/*7*/             { 2.491,2.246,1.968,1.655,1.304,0.913,0.479,0,-0.528,-1.107,-1.74,-2.431},
/*8*/             { 2.975,2.735,2.462,2.155,1.809,1.424,0.996,0.522,0,-0.573,-1.201,-1.887},
/*9*/             { 3.495,3.261,2.994,2.692,2.353,1.974,1.552,1.084,0.568,0,-0.622,-1.301},
/*10*/            { 4.053,3.825,3.565,3.27,2.937,2.564,2.148,1.687,1.177,0.616,0,-0.673},
/*11*/            { 4.651,4.43,4.177,3.888,3.562,3.196,2.787,2.332,1.829,1.275,0.666,0},
/*12*/            { 5.29,5.076,4.83,4.549,4.231,3.872,3.47,3.023,2.527,1.98,1.379,0.72}
};

Ultrasonic urR(7);
Ultrasonic urL(6);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);     // set baud rate to 57600bps for printing values at serial monitor.
  one.spiConnect(SSPIN);   // start SPI communication module
  brm.i2cConnect(MODULE_ADDRESS);   //Enable I2C communication
  brm.setModuleAddress(0x2C);       //Change I2C Address
  brm.setSonarStatus(ENABLE);       //Enable/Disable Sonar scanning
  brm.setRgbStatus(ENABLE);
  one.stop();

  analogReference(INTERNAL);//set the refenrence voltage 1.1V,the distinguishability can up to 1mV.

}

int phase = 1; //1 ist discovery mode, 2 ist logical search

void loop() {
  //move step
  one.move(20,45);
  delay(5000);
  int distR = int(urR.MeasureInCentimeters());
  int distL = int(urL.MeasureInCentimeters());
  int distC = getCDist();
  if(distR>20){
    turnright();
    one.lcd1("Rechts drehen");
    one.lcd1("");
    distC = getCDist();
    if(distC<60){
      moveOneSquare();
    }else{
      while(distC<60){
        one.move(10,24);
      }
      one.stop();
    }
  }else{
    distC = getCDist();
    if(distC<60){
      moveOneSquare();
    }else{
      while(distC<60){
        one.move(10,24);
      }
      one.stop();
    }
  }
  
}

//Ein Feld nach vorne bewegen
void moveOneSquare(){
  //Sensoren einlesen
  int now = getCDist();
  int finish = now - 30; //Feldgröße = 30 cm
  one.lcd1(finish);
  one.lcd2(now);
  delay(1000);
  while(true){
    //aktualisieren der Sensoren
    byte nowC=0;
    brm.readSonars(&sonarL,&nowC,&sonarR);
    now = int(nowC);
    one.lcd1(finish);
    one.lcd2(now);
    if(now<=finish || now<=5){
      one.stop();
      break;
    }
    one.move(10,20);
  }
  one.stop();
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
    if(now<=finish || now<=5){
      break;
    }
  }
  one.stop();
}

int getCDist(){
  byte nowC=0;
  brm.readSonars(&sonarL,&nowC,&sonarR);
  int distC = int(nowC);
  return distC;
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



float binSearch(long x)// this function used for measure the surrounding temperature
{
  int low,mid,high;
  low=0;
  //mid=0;
  high=100;
  while (low<=high)
  {
    mid=(low+high)/2;
    if(x<res[mid])
      low= mid+1;
    else//(x>res[mid])
      high=mid-1;
  }
  return mid;
}

float arraysearch(float x,float y)//x is the surrounding temperature,y is the object temperature
{
  int i=0;
  float tem_coefficient=100;//Magnification of 100 times  
  i=(x/10)+1;//Ambient temperature      
  voltage=(float)y/tem_coefficient;//the original voltage   
  //Serial.print("sensor voltage:\t");    
  //Serial.print(voltage,5);  
  //Serial.print("V");      
  for(temp3=0;temp3<13;temp3++)   
  {     
    if((voltage>obj[temp3][i])&&(voltage<obj[temp3+1][i]))        
    {     
      return temp3;         
    }     
  }
}
float measureSurTemp()
{  
  unsigned char i=0;
  float current_temp1=0;    
  int signal=0;   
  tempValue=0;

  for(i=0;i<10;i++)       //    
  {     
    tempValue+= analogRead(SUR_TEMP_PIN);       
    delay(10);    
  }   
  tempValue=tempValue/10;   
  temp = tempValue*1.1/1023;    
  R=2000000*temp/(2.50-temp);   
  signal=binSearch(R);    
  current_temp=signal-1+temp_calibration+(res[signal-1]-R)/(res[signal-1]-res[signal]);
  Serial.print("Surrounding temperature:");
  Serial.print(current_temp);
  return current_temp;
}

float measureObjectTemp()
{
  unsigned char i=0;  
  unsigned char j=0;  
  float sur_temp=0;  
  unsigned int array_temp=0;  
  float temp1,temp2; 
  float final_temp=0;
  objtValue=0;  
  for(i=0;i<10;i++)
  {
    objtValue+= analogRead(OBJ_TEMP_PIN); 
    delay(10); 
    }       
  objtValue=objtValue/10;//Averaging processing     
  temp1=objtValue*1.1/1023;//+objt_calibration; 
  sur_temp=temp1-(reference_vol+offset_vol);             
  Serial.print("\t Sensor voltage:");   
  Serial.print(sur_temp,3); 
  Serial.print("V");  
  array_temp=arraysearch(current_temp,sur_temp*1000);        
  temp2=current_temp;        
  temp1=(temperature_range*voltage)/(obj[array_temp+1][(int)(temp2/10)+1]-obj[array_temp][(int)(temp2/10)+1]);        
  final_temp=temp2+temp1;        
  if((final_temp>100)||(final_temp<=-10))
    {
    Serial.println ("\t out of range!");
    }
  else
    {
      Serial.print("\t object temperature:");   
      Serial.println(final_temp,2); 
      }
  return final_temp;
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

int TurnArray [4] = {0,0,0,0};
void directions(){
  for (int i=0; i<4; i++){
    while (true){
        if (one.readButton() == 1){
          TurnArray[i] = read_bearing();
          break;
          }
      }
      delay (500);
    }
  one.lcd1(TurnArray[0], TurnArray[1], TurnArray[2], TurnArray[3]);
}



int count = 1;
void turnright(){
  int nowBearing = read_bearing();
  int finishBearing = TurnArray[count];
  while (nowBearing < finishBearing){
    nowBearing = read_bearing();
    one.move(15,-15);
    one.lcd2(nowBearing, finishBearing);
    }
  one.stop();
  count = count + 1;
  if (count == 4){
    count = 0;
    }
}


