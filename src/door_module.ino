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

uint8_t S1_busy = 0;         //Is stepper moving
uint16_t S1_pulsetime = 250; //global speed, needed for calibration

//Stepper 2
const uint8_t S2_dir	=	10;
const uint8_t S2_step	=	11; //port PA16
const uint8_t S2_EN		=	12;
const uint8_t S2_CS		=	1;
const uint8_t S2_microsteps = 32;

uint8_t S2_busy = 0;      //Is stepper moving

//IR Sensor Pins
const uint8_t IR_top = A2;
const uint8_t IR_upper = 9;
const uint8_t IR_middle = A4;
const uint8_t IR_lower = A5;
const uint8_t IR_bottom = A3;

const uint8_t IR_barrier_rx = A1;
const uint8_t IR_barrier_tx = A0;

const uint8_t IR_all[7] = {IR_top, IR_upper, IR_middle, IR_lower, IR_bottom, IR_barrier_rx, IR_barrier_tx}; //1=blocked
volatile uint8_t IR_state[7] = {0,0,0,0,0,0,0};

//to improve function readability
const uint8_t top = 0;    //IR LEDs
const uint8_t upper = 1;
const uint8_t middle = 2;
const uint8_t lower = 3;
const uint8_t bottom = 4;
const uint8_t rx = 5;
const uint8_t tx = 6;

const uint8_t up = 0;   //door direction
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

//global variable to move the blocking stepper movement out of the i2c function
uint16_t Q_movesimple[4]; //queue a movesimple command to release I2C bus

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
//  attachInterrupt(digitalPinToInterrupt(IR_barrier_rx), IR_barrier_rx_ISR, RISING); //IR stepper side
//  attachInterrupt(digitalPinToInterrupt(IR_barrier_tx), IR_barrier_tx_ISR, RISING);
  
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
  Wire.onRequest(sendData);     //what to do when being talked to
  Wire.onReceive(receiveEvent); //what to do when/with data received
  
  //----- calibrate movement timings
  //open first
  //movesimple(up,top,200);
  //calibrate(S1_pulsetime);
  //delay(1000);
}

void loop()
{
  //read IR sensors
  // IR_state[0] = digitalRead(top);
  // IR_state[1] = digitalRead(upper);
  // IR_state[2] = digitalRead(middle);
  // IR_state[3] = digitalRead(lower);
  // IR_state[4] = digitalRead(bottom);
  
  //perform queued moves
  if(Q_movesimple[0])
  {
    movesimple(Q_movesimple[1],Q_movesimple[2],Q_movesimple[3]);
    Q_movesimple[0] = 0;
  }
  
}

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//close step by step with timing and feedback ----------------------------------
//calibrate necessary, uses Irs parallel to tube for feedback move
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
//move door to target position, if door is blocked will retry movement
void movesimple(uint8_t direction, uint8_t target, uint16_t pulsetime)
{
  S1_busy = 1;
  digitalWrite(S1_dir, direction); //0 open, 1 close
  
  uint8_t temp_target; //temporary target to coordinate retries
  temp_target = target;
  uint8_t done = 0;

  while(!done)
  {
    //read IR states
    IR_state[rx] = digitalRead(IR_barrier_rx);
    IR_state[tx] = digitalRead(IR_barrier_tx);
    IR_state[temp_target] = digitalRead(IR_all[temp_target]);
    
    //if barrier is blocked on moving down, change direction and target to retry target
    if(direction && (IR_state[rx] || IR_state[tx]))
    {
      direction = up;                  //change dir to up
      digitalWrite(S1_dir, direction); //0 up, 1 down
      temp_target = top;               //retry target is top <-- can be changed for different target e.g. only one step up, or half open
    }
    
    //if we are at the retry target after a retry and not blocked, move back to original target
    if(!(IR_state[temp_target] ^ direction) && (target != temp_target) && !IR_state[rx] && !IR_state[tx])
    {
      direction = down;                  //change dir to up
      digitalWrite(S1_dir, direction);   //0 up, 1 down
      temp_target = target;
    }
    
    //if not at defined target, move
    if(IR_state[temp_target] ^ direction)
    {
      move(pulsetime);
    }
    
    //if we have reached our target we are done
    if(!(IR_state[temp_target] ^ direction) && (target == temp_target))
    {
      done = 1;
    }
  }
  
  //done with movement
  S1_busy = 0;
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
//move two microsteps
void move(uint16_t pulsetime)
{
  REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
  delayMicroseconds(pulsetime);
  REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
  delayMicroseconds(pulsetime);
}

//I2C receive instructions
void receiveEvent(int bytes_incoming)
{
  uint8_t inputbuffer[7] = {0,0,0,0,0,0,0};
//  uint8_t stepper;
//  int32_t steps = 0;
//  uint8_t dir;
//  uint8_t speed;
  
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
    //driver 1 rms
    //driver 1 holdcurrent
    //driver 1 speed
    //driver 2 rms
    //driver 2 holdcurrent
    //driver 2 speed
  }
  
  if(option == 1) //calibrate
  {
    S1_pulsetime = S1_pulsetime | inputbuffer[2];
    S1_pulsetime = (S1_pulsetime << 8) | inputbuffer[1];
    
    calibrate(S1_pulsetime);
  }
  
  if(option == 2) //move door with simple feedback
  {
    Q_movesimple[1] = inputbuffer[1];                           //direction
    Q_movesimple[2] = inputbuffer[2];                           //target
    Q_movesimple[3] = 0;                                       //clear
    Q_movesimple[3] = Q_movesimple[3] | inputbuffer[4];         //pulsetime
    Q_movesimple[3] = (Q_movesimple[3] << 8) | inputbuffer[3];  //pulsetime
    
    Q_movesimple[0] = 1; //queues movement
  }
  
  if(option == 3) //close door with fancy stepwise feedback
  {
    uint8_t start = inputbuffer[1];
    uint8_t stop = inputbuffer[2];
    
    closefancy(start, stop, S1_pulsetime);
  }
}

void sendData()
{
  uint8_t sendbuffer[2] = {0,0}; //buffer for sending data over I2C
  //potentially send different datapacks
  //  if(selectdata == x)
  //  {sendbuffer1 || sendbuffer2}
  
  //sendbuffer[0] IR status
  sendbuffer[0] = IR_state[top] & 0x01;
  sendbuffer[0] = (sendbuffer[0] << 1) | (IR_state[upper] & 0x01);
  sendbuffer[0] = (sendbuffer[0] << 1) | (IR_state[middle] & 0x01);
  sendbuffer[0] = (sendbuffer[0] << 1) | (IR_state[lower] & 0x01);
  sendbuffer[0] = (sendbuffer[0] << 1) | (IR_state[bottom] & 0x01);
  sendbuffer[0] = (sendbuffer[0] << 1) | (IR_state[rx] & 0x01);
  sendbuffer[0] = (sendbuffer[0] << 1) | (IR_state[tx] & 0x01);
  
  sendbuffer[1] = S1_busy & 0x01;
  
  Wire.write(sendbuffer,2);
}
