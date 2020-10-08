#include <Arduino.h>
#include <Wire.h>
#include <TMC2130Stepper.h>
#include <Adafruit_DotStar.h>

//Stepper         //mopss2//mopss1//itsybitsy
const int S_dir1	=	A2;
const int S_step1	=	A3;
const int S_EN1		=	A5;
const int S_CS1		=	A4;
const int microsteps1 = 32;

//34uS minimum delay
//float b = 100000/microsteps1;
//int n = 1;
//int maxrpm = 0;
//unsigned long t;
//bool toggle;

Adafruit_DotStar strip(1, 41, 40, DOTSTAR_BRG); //create dotstar object

TMC2130Stepper driver1 = TMC2130Stepper(S_EN1, S_dir1, S_step1, S_CS1);

void setup()
{
  //Set up RGB LED on board, and turn it off
  strip.begin(); //Initialize pins for output
  strip.show();  //Turn all LEDs off ASAP

  //Serial.begin(115200);
  //while(!Serial);

  //Setup stepper driver
  driver1.begin(); //initialize pins and registries
	driver1.rms_current(200); //set driving current RMS (300max for NEMA 8) // 100
  driver1.stealthChop(1); //enable stealthchopping for quiet runnning
	driver1.microsteps(microsteps1); //default 256 for quiet running
  driver1.interpolate(1);
  driver1.double_edge_step(1);
  //holdcurrent

	digitalWrite(S_EN1,LOW); //Enable output on drivers

  //start I2C on address 16
  Wire.begin(0x11); //atsamd cant multimaster
  Wire.onReceive(receiveEvent); //what to do when/with data received
}

void loop()
{
  //508 nS
  //REG_PORT_OUTSET0 = PORT_PA23;
  //REG_PORT_OUTCLR0 = PORT_PA23;
/*
  //lookuptable
  for(int i = 0; i < (200*64); i++)
  {

    delayMicroseconds(lookuptable[i]*10);
    REG_PORT_OUTSET0 = PORT_PA23;
    delayMicroseconds(lookuptable[i]*10);
    REG_PORT_OUTCLR0 = PORT_PA23;
  }
  for(int i = (200*64); i > 0; i--)
  {
    delayMicroseconds(lookuptable[i]*10);
    REG_PORT_OUTSET0 = PORT_PA23;
    delayMicroseconds(lookuptable[i]*10);
    REG_PORT_OUTCLR0 = PORT_PA23;
  }
*/
/*
b = 10000;
n = 1;
for(int i = 0; i < 10000; i++)
{
  delayMicroseconds(b);
  REG_PORT_OUTSET0 = PORT_PA23;
  b = b - ((2 * b) / (4 * n + 1));
  n++;
  // if(b <= (1000 / microsteps1))
  // {
  //   b = (1000 / microsteps1);
  // }

  delayMicroseconds(b);
  REG_PORT_OUTCLR0 = PORT_PA23;
  b = b - ((2 * b) / (4 * n + 1));
  n++;
//  if(b <= (1000 / microsteps1))
//  {
//    b = (1000 / microsteps1);
//  }
}
*/
/*
if(micros() - t >= b)
{
  t = micros();

  b = b - ((2 * b) / (4 * n + 1));
  n = n + 1;


  if(toggle == 1)
  {
    toggle = 0;
    REG_PORT_OUTSET0 = PORT_PA23;
  }
  else
  {
    toggle = 1;
    REG_PORT_OUTCLR0 = PORT_PA23;
  }
}
*/
}

//I2C receive instructions
void receiveEvent(int bytes_incoming)
{
  uint8_t inputbuffer[6] = {0,0,0,0,0,0};
  uint32_t steps = 0;
  uint8_t dir;
  uint8_t speed;

  for(int i = 0; i < bytes_incoming; i++)
  {
    byte c = Wire.read();
    inputbuffer[i] = c;
  }

  steps = steps | inputbuffer[3];
  steps = (steps << 8) | inputbuffer[2];
  steps = (steps << 8) | inputbuffer[1];
  steps = (steps << 8) | inputbuffer[0];

  dir = inputbuffer[4];
  speed = inputbuffer[5];

  //set direction pin
  digitalWrite(S_dir1, dir);

  //rotates motor
  for(int i = 0; i < steps; i++)
  {
//    REG_PORT_OUTSET0 = PORT_PA23; // ~0.4us mopss 1
    REG_PORT_OUTSET0 = PORT_PA04; // ~0.4us    mopss 2
    delayMicroseconds(speed);
//    REG_PORT_OUTCLR0 = PORT_PA23; // ~0.4us mopss 1
    REG_PORT_OUTCLR0 = PORT_PA04; // ~0.4us  mopss 2
    delayMicroseconds(speed);
  }
}
