/* 
 This example was created by José Cruz on October 2016 
 
 This code example is in the public domain. 
 http://www.botnroll.com

Seguimento de linha:
Os motores variam com a linha de acordo com uma função quadrática.
Botão 3 (PB3) entra no menu de configuração do controlo.

https://www.desmos.com/calculator

*/
#include <BnrOneA.h>   // Bot'n Roll ONE A library
#include <EEPROM.h>    // EEPROM reading and writing
#include <SPI.h>       // SPI communication library required by BnrOne.cpp
BnrOneA one;           // declaration of object variable to control the Bot'n Roll ONE A

//constants definitions
#define SSPIN  2    //Slave Select (SS) pin for SPI communication
#define M1  1       //Motor1
#define M2  2       //Motor2

float batmin=10.5;  // safety voltage for discharging the battery

int vel=55;          
double kLimit=40.0;
int vCurve=12;        

int Vtrans=50;  //
int SV1=60, SV2=60;  //Sensores virtuais para os limites do seguidor de linha.

void setup() 
{  
  Serial.begin(57600);     // sets baud rate to 57600bps for printing values at serial monitor.
  one.spiConnect(SSPIN);   // starts the SPI communication module
  one.stop();              // stops all move
  one.minBat(batmin);      // safety voltage for discharging the battery
  delay(1000);
}

void loop() 
{
  int velM1=0, velM2=0;
  int line=one.readLine();
//  Serial.print(" Line:");Serial.print(line); 
  if(line<=0)
  {
      velM1=(int)((double)vel*cos(((double)line/kLimit)));
      velM2=vel+(vel-velM1);
  }
  else
  {
      velM2=(int)((double)vel*cos(((double)line/kLimit)));
      velM1=vel+(vel-velM2);
  }
//Limitar mínimos e máximos da velocidade dos motores

  if(velM1<-1)
      velM1=-1;
  if(velM2<-1)
      velM2=-1;
  if(velM1>vel+vCurve)
      velM1=vel+vCurve;
  if(velM2>vel+vCurve)
      velM2=vel+vCurve;

// Serial.print("   M1:");  Serial.print(velM1); Serial.print("   M2:"); Serial.println(velM2);  
  one.move(velM1,velM2);
  
  //Menus de configuração
  if(one.readButton()==3) menu();// Entra no menu
}

void menu()
{
    int var=0;
    static int butt=3;
    float temp=0.0;
    one.stop();
    one.lcd1("  Menu Config:");
    while(butt!=0) //Espera que se largue o botão 3
    {
      butt=one.readButton();
      delay(50);
    }
    
    //*************** Velociade Maxima ***************
    var=vel;
    while(butt!=3)
    {
        one.lcd2("    VelMax:",var);
        butt=one.readButton();
        if(butt==1)
        {
          var++;
          delay(150);
        }
        if(butt==2)
        {
          var--;
          delay(150);
        }
    }
    while(butt==3) //Espera que se largue o botão 3
    {butt=one.readButton();}
    vel=var;        

    //*************** Incremento da roda que acelera ***************
    var=vCurve;
    while(butt!=3)
    {
        one.lcd2("Curve Boost:",var);
        butt=one.readButton();
        if(butt==1)
        {
          var++;
          delay(150);
        }
        if(butt==2)
        {
          var--;
          delay(150);
        }
    }
    while(butt==3) //Espera que se largue o botão 3
    {butt=one.readButton();}
    vCurve=var;  

     //*************** Ganho KLimite ***************
    temp=kLimit*1.0;
    var=(int)temp;
    while(butt!=3)
    {
        one.lcd2(" Line Gain:",var);
        butt=one.readButton();
        if(butt==1)
        {
          var+=1;
          delay(150);
        }
        if(butt==2)
        {
          var-=1;
          delay(150);
        }
    }
    while(butt==3) //Espera que se largue o botão 3
    {butt=one.readButton();}
    kLimit=(float)var/1.0;  
    
    
    //*************** Termina Configuração *************** 
    one.lcd1("Line  Following!");
    one.lcd2("www.botnroll.com");
    delay(250);
}
