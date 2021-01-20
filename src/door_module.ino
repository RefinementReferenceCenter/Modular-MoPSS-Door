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
const uint8_t IR_top = A3;
const uint8_t IR_upper = A5;
const uint8_t IR_middle = A4;
const uint8_t IR_lower = 9;
const uint8_t IR_bottom = A2;

const uint8_t IR_barrier_rx = A1;
const uint8_t IR_barrier_tx = A0;

//LEDs
const uint8_t LED1 = 7;
const uint8_t LED2 = 5;

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
  strip.show();  //Turn all LEDs off ASAP

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
	driver1.rms_current(100,0.11,0.5); //set driving current RMS (current RMS, Rsens, hold multiplier) (300max for NEMA 8)
  driver1.stealthChop(1); //enable stealthchopping for quiet runnning
	driver1.microsteps(S1_microsteps); //default 256 for quiet running
  driver1.interpolate(1);
  driver1.double_edge_step(1);

  driver2.begin(); //initialize pins and registries
	driver2.rms_current(100,0.11,0.5); //set driving current RMS (current RMS, Rsens, hold multiplier) (300max for NEMA 8)
  driver2.stealthChop(1); //enable stealthchopping for quiet runnning
	driver2.microsteps(S2_microsteps); //default 256 for quiet running
  driver2.interpolate(1);
  driver2.double_edge_step(1);

	digitalWrite(S1_EN,LOW); //Enable output on drivers
  digitalWrite(S2_EN,LOW); //Enable output on drivers
  
  //start I2C on address 16
  Wire.begin(0x10); //atsamd cant multimaster
  Wire.onReceive(receiveEvent); //what to do when/with data received
}

void loop()
{
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
  
}

//I2C receive instructions
void receiveEvent(int bytes_incoming)
{
  uint8_t inputbuffer[7] = {0,0,0,0,0,0,0};
  uint8_t stepper;
  uint32_t steps = 0;
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
