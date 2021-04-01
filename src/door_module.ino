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

uint8_t sendbuffer[10]; //buffer for sending data over I2C

//Stepper 1
const uint8_t S1_dir	=	0;
const uint8_t S1_step	=	4; //port PA08
const uint8_t S1_EN		=	2;
const uint8_t S1_CS		=	3;
const uint8_t S1_microsteps = 32;

uint16_t S1_pulsetime = 250; //global speed, needed for calibration

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

//to improve function readability
const uint8_t top = 0;
const uint8_t upper = 1;
const uint8_t middle = 2;
const uint8_t lower = 3;
const uint8_t bottom = 4;

const uint8_t up = 0;
const uint8_t down = 1;

//LEDs
const uint8_t LED1 = 7;
const uint8_t LED2 = 5;

uint8_t closedoor = 1;

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
  
  //setup Interrupts
  attachInterrupt(digitalPinToInterrupt(IR_barrier_rx), IR_barrier_ISR, HIGH); //IR stepper side
  attachInterrupt(digitalPinToInterrupt(IR_barrier_tx), IR_barrier_ISR, HIGH); //IR stepper side
  
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
  movesimple(up,top,200);
  calibrate(S1_pulsetime);
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
  
  //opensimple(200);
  movesimple(up,top,S1_pulsetime);
  
  delay(1000);
  
  closefancy(top,bottom,S1_pulsetime); //start, stop

  delay(1000);
  //-----

  
}

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//close step by step with timing and feedback ----------------------------------
void closefancy(uint8_t start, uint8_t stop, uint16_t pulsetime)
{
  uint8_t closing;
  uint8_t lower_door = 1;
  uint32_t move_time;

  //calculate total move time
  // for(int i = start; i < stop; i++)
  // {
  //   total_move_time += move_interval_times[i];
  // }
  
  //move through all IR barrier intervals step by step
  for(int i = start; i < stop; i++)
  {
    //close step by step with feedback
    closing = 1;
    while(closing)
    {
      //lower door from start to stop
      digitalWrite(S1_dir, down); //move down
      move_time = millis();
      while((digitalRead(IR_all[i+1]) == 0) && lower_door)
      {
        if((millis() - move_time) < (move_interval_times[i] * 1.2))
        {
          move(pulsetime);
        }
        else
        {
          lower_door = 0;
          //delay(1000); //wait before moving up again?
        }
      }
      //move up to reset if door doesn't reach target within time
      //Problem: If door is already above sensor, it will start moving down again, potentially
      //unravelling the string
      //solution 1: always move all the way up
      //solution 2: use movement timing <- now
      digitalWrite(S1_dir, up); //move up
      move_time = millis();
      //even if IR is already unblocked (mouse lifts door up) move back a bit to prevent string from uncurling
      while(((digitalRead(IR_all[i]) == 1) || ((millis() - move_time) < (move_interval_times[i] * 1.1))) && !lower_door)
      {
        move(pulsetime);
      }
      //if lower_door true, move finished, else door moved up, try again
      if(lower_door)
      {
        closing = 0;
      }
      lower_door = 1;
    }
  }
}

//------------------------------------------------------------------------------
void movesimple(uint8_t direction, uint8_t target, uint16_t pulsetime)
{
  digitalWrite(S1_dir, direction); //0 open, 1 close
  //
  while(digitalRead(IR_all[target]) ^ direction)
  {
    move(pulsetime);
  }
}

//------------------------------------------------------------------------------
void calibrate(uint16_t pulsetime)
{
  movesimple(up, top, pulsetime);
  //now move down
  digitalWrite(S1_dir, down);
  uint32_t move_time = millis();
  while(digitalRead(IR_upper) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);
  }
  move_interval_times[0] = millis() - move_time;
  delay(100);

  move_time = millis();
  while(digitalRead(IR_middle) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);
  }
  move_interval_times[1] = millis() - move_time;
  delay(100);
  
  move_time = millis();
  while(digitalRead(IR_lower) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);
  }
  move_interval_times[2] = millis() - move_time;
  delay(100);
  
  move_time = millis();
  while(digitalRead(IR_bottom) == 0)
  {
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);
  }
  move_interval_times[3] = millis() - move_time;
  delay(100);
  
  movesimple(up, top, pulsetime);
  
  // Serial.print(move_interval_times[0]);
  // Serial.print(" ");
  // Serial.print(move_interval_times[1]);
  // Serial.print(" ");
  // Serial.print(move_interval_times[2]);
  // Serial.print(" ");
  // Serial.println(move_interval_times[3]);
}

//------------------------------------------------------------------------------
void move(uint16_t pulsetime)
{
  REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
  delayMicroseconds(pulsetime);
  REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
  delayMicroseconds(pulsetime);
}

void IR_barrier_ISR()
{
  //pause everything, wait for barrier to clear
  while(digitalRead(IR_barrier_rx) || digitalRead(IR_barrier_tx))
  {
    if(digitalRead(IR_barrier_rx))
    {
      digitalWrite(LED1,HIGH);
    }
    else
    {
      digitalWrite(LED1,LOW);
    }
    
    if(digitalRead(IR_barrier_tx))
    {
      digitalWrite(LED2,HIGH);
    }
    else
    {
      digitalWrite(LED2,LOW);
    }
  }
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
}


//I2C receive instructions
void receiveEvent(int bytes_incoming)
{
  uint8_t inputbuffer[7] = {0,0,0,0,0,0,0};
  uint8_t stepper;
  int32_t steps = 0;
  uint8_t dir;
  uint8_t speed;
  
  uint8_t option;
  
  //collect bytes from I2C
  for(uint8_t i = 0; i < bytes_incoming; i++)
  {
    uint8_t c = Wire.read();
    inputbuffer[i] = c;
  }
  
  //----- process received data -----
  option = inputbuffer[0];
  
  if(option == 0) //set config
  {
    //write config for various door functions
    
    
  }
  
  if(option == 1) //calibrate
  {
    S1_pulsetime = S1_pulsetime | inputbuffer[2];
    S1_pulsetime = (S1_pulsetime << 8) | inputbuffer[1];
    
    calibrate(S1_pulsetime);
  }
  
  if(option == 2) //move door with simple feedback
  {
    uint8_t direction = inputbuffer[1];
    uint8_t target = inputbuffer[2];
    uint16_t pulsetime;
    pulsetime = pulsetime | inputbuffer[4];
    pulsetime = (pulsetime << 8) | inputbuffer[3];
    
    movesimple(direction,target,pulsetime);
  }
  
  if(option == 3) //close door with fancy stepwise feedback
  {
    uint8_t start = inputbuffer[1];
    uint8_t stop = inputbuffer[2];
    
    closefancy(start, stop, S1_pulsetime);
  }
  
  
  // //create 32bit variable from 4 bytes
  // steps = steps | inputbuffer[3];
  // steps = (steps << 8) | inputbuffer[2];
  // steps = (steps << 8) | inputbuffer[1];
  // steps = (steps << 8) | inputbuffer[0];
  //
  // dir = inputbuffer[4];
  // speed = inputbuffer[5];
  // stepper = inputbuffer[6];
  
  //select stepper explicitly
  // if(stepper == 1)
  // {
  //   //set direction
  //   digitalWrite(S1_dir, dir);
  //
  //   //rotates stepper 1
  //   for(int i = 0; i < steps; i++)
  //   {
  //     REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
  //     delayMicroseconds(speed);
  //     REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
  //     delayMicroseconds(speed);
  //   }
  // }
  // if(stepper == 2)
  // {
  //   //set direction
  //   digitalWrite(S2_dir, dir);
  //
  //   //rotates stepper 2
  //   for(int i = 0; i < steps; i++)
  //   {
  //     REG_PORT_OUTSET0 = PORT_PA16; // ~0.4us stepper 1
  //     delayMicroseconds(speed);
  //     REG_PORT_OUTCLR0 = PORT_PA16; // ~0.4us
  //     delayMicroseconds(speed);
  //   }
  // }
}

void sendData()
{
//potentially send different datapacks
//  if(selectdata == x)
//  {sendbuffer1 || sendbuffer2  }
  
  //sendbuffer[1-6] IR status
  for(uint8_t i = 0; i < 7; i++)
  {
    sendbuffer[i] = digitalRead(IR_all[i]);
  }
  
  //sendbuffer[7] door status
  //sendbuffer[7] =
  //different statuses, moving, finished, time etc.?
  
  //sendbuffer[x]
  //status of stepper 2?
  
  
  Wire.write(sendbuffer,10);
  
  //clear sendbuffer after send
  for(int i = 0;i < 10;i++)
  {
    sendbuffer[i] = 0;
  }
}
