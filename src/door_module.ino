//------------------------------------------------------------------------------
/*
--- Adafruit ItsyBitsy M0 pin mapping - Hardware Revision v6.0 ---

 A0- IR TX Barrier
 A1- IR RX Barrier
 A2- IR Bottom
 A3- IR Top
 A4- IR Middle
 A5- IR Upper

 D0- Stepper 1 Direction
 D1- Stepper 2 Chip Select (solder for HW v6.0)
 D2- Stepper 1 Enable
 D3- Stepper 1 Chip Select
 D4- Stepper 1 Step [PA08]
 D5- LED 2
 D7- LED 1
 D9- IR Lower
D10- Stepper 2 Direction
D11- Stepper 2 Step [PA16]
D12- Stepper 2 Enable
D13-

D22- I2C SDA
D23- I2C SCL

--- Experimental Setup ---

   | door
   |
   |
|[ | <| IR top
|[ | <| IR upper
|[ | <| IR middle
|[   <| IR lower
|[   <| IR bottom

*/

#include <Arduino.h>
#include <Wire.h>
#include <TMC2130Stepper.h>
#include <Adafruit_DotStar.h>

//Stepper 1
const uint8_t S1_dir	=	0;
const uint8_t S1_step	=	4; //port PA08
const uint8_t S1_EN		=	2;
const uint8_t S1_CS		=	3;
const uint8_t S1_microsteps = 32;

//Stepper 2
const uint8_t S2_dir	=	10;
const uint8_t S2_step	=	11; //port PA16
const uint8_t S2_EN		=	12;
const uint8_t S2_CS		=	1;
const uint8_t S2_microsteps = 32;

//IR Sensors
const uint8_t IR_top = A2;
const uint8_t IR_upper = 9;
const uint8_t IR_middle = A4;
const uint8_t IR_lower = A5;
const uint8_t IR_bottom = A3;

const uint8_t IR_barrier_rx = A1;
const uint8_t IR_barrier_tx = A0;

const uint8_t IR_all[7] = {IR_top, IR_upper, IR_middle, IR_lower, IR_bottom, IR_barrier_rx, IR_barrier_tx};

//LEDs
const uint8_t LED1 = 7;
const uint8_t LED2 = 5;

uint8_t closedoor = 1;
uint32_t move_time1;

uint32_t topup_time;
uint32_t upmid_time;
uint32_t midlow_time;
uint32_t lowbot_time;

uint32_t move_interval_times[4];

//34uS minimum delay
//float b = 100000/microsteps1;
//int n = 1;
//int maxrpm = 0;
//unsigned long t;
//bool toggle;

Adafruit_DotStar strip(1, 41, 40, DOTSTAR_BRG); //create dotstar object

TMC2130Stepper driver1 = TMC2130Stepper(S1_EN, S1_dir, S1_step, S1_CS);
TMC2130Stepper driver2 = TMC2130Stepper(S2_EN, S2_dir, S2_step, S2_CS);

void setup()
{
  //Set up RGB LED on board, and turn it off
  strip.begin(); //Initialize pins for output
  strip.show();  //Turn dotstar LED off

  //while(!Serial);
  
  //Setup IR Sensors (high when interrrupted)
  pinMode(IR_top,INPUT);
  pinMode(IR_upper,INPUT);
  pinMode(IR_middle,INPUT);
  pinMode(IR_lower,INPUT);
  pinMode(IR_bottom,INPUT);
  
  pinMode(IR_barrier_rx,INPUT);
  pinMode(IR_barrier_tx,INPUT);
  
  //Setup LEDs
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  
  //Setup stepper drivers
  driver1.begin(); //initialize pins and registries
	driver1.rms_current(80,0.11,0.4); //set driving current RMS (current RMS, Rsens, hold multiplier) (300max for NEMA 8)
  driver1.stealthChop(1); //enable stealthchopping for quiet runnning
	driver1.microsteps(S1_microsteps); //default 256 for quiet running
  driver1.interpolate(1);
  driver1.double_edge_step(1);

  driver2.begin(); //initialize pins and registries
	driver2.rms_current(80,0.11,0.4); //set driving current RMS (current RMS, Rsens, hold multiplier) (300max for NEMA 8)
  driver2.stealthChop(1); //enable stealthchopping for quiet runnning
	driver2.microsteps(S2_microsteps); //default 256 for quiet running
  driver2.interpolate(1);
  driver2.double_edge_step(1); //step on rising and falling edge

	digitalWrite(S1_EN,LOW); //Enable output on drivers
  digitalWrite(S2_EN,LOW); //Enable output on drivers
  
  //start I2C on address 16
  Wire.begin(0x10); //atsamd cant multimaster
  Wire.onReceive(receiveEvent); //what to do when/with data received
  
  //----- calibrate movement timings
  //open first
  opensimple();
  calibrate();
  delay(1000);
}

void loop()
{
  // digitalWrite(LED1,HIGH);
  //
  // Serial.print("IRs: ");
  // Serial.print(digitalRead(IR_top));
  // Serial.print("-");
  // Serial.print(digitalRead(IR_upper));
  // Serial.print("-");
  // Serial.print(digitalRead(IR_middle));
  // Serial.print("-");
  // Serial.print(digitalRead(IR_lower));
  // Serial.print("-");
  // Serial.print(digitalRead(IR_bottom));
  // Serial.print("-");
  // Serial.print(digitalRead(IR_barrier_rx));
  // Serial.print("-");
  // Serial.println(digitalRead(IR_barrier_tx));
  // //delay(1000);
  // digitalWrite(LED1,LOW);
  

  
  //-----
  
  opensimple();
  delay(1000);
  
  closefancy(IR_all[0],IR_all[1]);
  // uint8_t closing = 1;
  // uint8_t lower_door = 1;
  // uint32_t move_time;
  //
  // while(closing)
  // {
  //   //lower door from start to stop
  //   move_time = millis();
  //   digitalWrite(S1_dir, 1);
  //   while((digitalRead(IR_upper) == 0) && lower_door)
  //   {
  //     if(millis() - move_time < 1000)
  //     {
  //       move();
  //     }
  //     else
  //     {
  //       lower_door = 0;
  //       //delay(1000); //wait?
  //     }
  //   }
  //   //move up to reset if door doesn't reach target within time
  //   digitalWrite(S1_dir, 0);
  //   while((digitalRead(IR_top) == 1) && !lower_door)
  //   {
  //     move();
  //   }
  //   //if lower_door true, move finished, else door moved up, try again
  //   if(lower_door)
  //   {
  //     closing = 0;
  //     digitalWrite(LED1,1);
  //     delay(250);
  //     digitalWrite(LED1,0);
  //   }
  //   lower_door = 1;
  // }
  
  
  
  
  delay(1000);
  //-----

  
}

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################


//close with timing
void closefancy(uint8_t IR_start, uint8_t IR_stop)
{
  uint8_t closing = 1;
  uint8_t lower_door = 1;
  uint32_t move_time;
  
  while(closing)
  {
    //lower door from start to stop
    move_time = millis();
    digitalWrite(S1_dir, 1);
    while((digitalRead(IR_stop) == 0) && lower_door)
    {
      if(millis() - move_time < move_interval_times[0] * 1.2)
      {
        move();
      }
      else
      {
        lower_door = 0;
        //delay(1000); //wait?
      }
    }
    //move up to reset if door doesn't reach target within time
    digitalWrite(S1_dir, 0);
    while((digitalRead(IR_start) == 1) && !lower_door)
    {
      move();
    }
    //if lower_door true, move finished, else door moved up, try again
    if(lower_door)
    {
      closing = 0;
    }
    lower_door = 1;
  }
}

//------------------------------------------------------------------------------
void opensimple()
{
  digitalWrite(S1_dir, 0);
  while(digitalRead(IR_top))
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(500);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(500);
  }
}

void closesimple()
{
  digitalWrite(S1_dir, 1);
  while(digitalRead(IR_bottom))
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(500);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(500);
  }
}

//------------------------------------------------------------------------------
void calibrate()
{
  //now move down
  digitalWrite(S1_dir, 1);
  move_time1 = millis();
  while(digitalRead(IR_upper) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(500);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(500);
  }
  move_interval_times[0] = millis() - move_time1;
  delay(200);

  move_time1 = millis();
  while(digitalRead(IR_middle) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(500);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(500);
  }
  move_interval_times[1] = millis() - move_time1;
  delay(200);
  
  move_time1 = millis();
  while(digitalRead(IR_lower) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(500);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(500);
  }
  move_interval_times[2] = millis() - move_time1;
  delay(200);
  
  move_time1 = millis();
  while(digitalRead(IR_bottom) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(500);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(500);
  }
  move_interval_times[3] = millis() - move_time1;
  
  Serial.print(move_interval_times[0]);
  Serial.print(" ");
  Serial.print(move_interval_times[1]);
  Serial.print(" ");
  Serial.print(move_interval_times[2]);
  Serial.print(" ");
  Serial.println(move_interval_times[3]);
}

//-----
void move()
{
  REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
  delayMicroseconds(500);
  REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
  delayMicroseconds(500);
}

//I2C receive instructions

void receiveEvent(int bytes_incoming)
{
  uint8_t inputbuffer[7] = {0,0,0,0,0,0,0};
  uint8_t stepper;
  int32_t steps = 0;
  uint8_t dir;
  uint8_t speed;
  
  //collect bytes from I2C
  for(uint8_t i = 0; i < bytes_incoming; i++)
  {
    byte c = Wire.read();
    inputbuffer[i] = c;
  }
  
  //create 32bit variable from 4 bytes
  steps = steps | inputbuffer[3];
  steps = (steps << 8) | inputbuffer[2];
  steps = (steps << 8) | inputbuffer[1];
  steps = (steps << 8) | inputbuffer[0];

  dir = inputbuffer[4];
  speed = inputbuffer[5];
  stepper = inputbuffer[6];
  
  //select stepper explicitly
  if(stepper == 1)
  {
    //set direction
    digitalWrite(S1_dir, dir);
    
    //rotates stepper 1
    for(int i = 0; i < steps; i++)
    {
      REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
      delayMicroseconds(speed);
      REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
      delayMicroseconds(speed);
    }
  }
  if(stepper == 2)
  {
    //set direction
    digitalWrite(S2_dir, dir);
    
    //rotates stepper 2
    for(int i = 0; i < steps; i++)
    {
      REG_PORT_OUTSET0 = PORT_PA16; // ~0.4us stepper 1
      delayMicroseconds(speed);
      REG_PORT_OUTCLR0 = PORT_PA16; // ~0.4us
      delayMicroseconds(speed);
    }
  }
}
