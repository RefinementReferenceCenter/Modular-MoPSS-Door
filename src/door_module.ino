/*------------------------------------------------------------------------------
- PJRC Teensy LC pin mapping - Hardware Revision v7.0

//Stepper 1
D1  - IR1 TX1          infrared lightbarrier 1 transmitter side, crossed-over input 1
D0  - IR1 TX2          infrared lightbarrier 1 transmitter side, crossed-over input 2
D21 - IR1 RX1          infrared lightbarrier 1 receiver side, crossed-over input 1
D24 - IR1 RX2          infrared lightbarrier 1 receiver side, crossed-over input 2
D25 - IR1 top          infrared lightbarrier 1 receiver side, perpendicular input 1
D4  - IR1 Bottom       infrared lightbarrier 1 receiver side, perpendicular input 2
D2  - Stepper 1 CS     Chip Select pin
D3  - Stepper 1 DIR    Direction signal
D22 - Stepper 1 STEP   Step pulse signal
D23 - Stepper 1 EN     enable signal
D14 - LED1             status LED for Stepper 1

//Stepper 2
D6 - IR2 TX1
D5 - IR2 TX2
D20 - IR2 RX1
D15 - IR2 RX2
D16 - IR2 Top
D17 - IR2 Bottom
D7  - Stepper 2 CS
D10 - Stepper 2 DIR
D8  - Stepper 2 STEP
D9  - Stepper 2 ENABLE
D26 - LED 2

//Comms
D11 - MOSI
D12 - MISO
D13 - SCK
D18 - SDA
D19 - SCL

--- Experimental Setup ---

   | door
   |
   |                     \    / IR R/TX 1
|[ | <| IR RX top         \  /
|  |  |                    \/
|  |  |                    /\
|     |                   /  \
|[   <| IR RX bottom     /    \ IR R/TX 2

*///----------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <TMC2130Stepper.h>
#include <Adafruit_DotStar.h>

const uint8_t has_lock = 0;

//Stepper 1
const uint8_t S1_dir	=	0;
const uint8_t S1_step	=	4; //port PA08
const uint8_t S1_EN		=	2;
const uint8_t S1_CS		=	3;
const uint8_t S1_microsteps = 32;

volatile uint8_t S1_busy = 0; //Is stepper moving
uint16_t S1_pulsetime = 250;  //global speed, for calibration movement
int16_t steps_to_close;       //holds the number of steps needed to go from open to close position of the door
int16_t steps_last_move;      //holds current position in steps when move is interrupted

//Stepper 2
const uint8_t S2_dir	=	10;
const uint8_t S2_step	=	11; //port PA16
const uint8_t S2_EN		=	12;
const uint8_t S2_CS		=	1;
const uint8_t S2_microsteps = 32;

uint8_t S2_busy = 0;      //Is stepper moving
uint8_t door_locked = 1;  //is door locked

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

const uint8_t close = 1; //lock direction
const uint8_t open = 0;
const uint16_t lock_move_steps = 192;

//LEDs
const uint8_t LED1 = 7;
const uint8_t LED2 = 5;

uint8_t closedoor = 1;

uint32_t move_interval_times[4];

//global variable to move the blocking stepper movement out of the i2c function
volatile uint16_t Q_movesimple[4]; //queue a movesimple command to release I2C bus
volatile uint16_t Q_vibrate[4];
volatile uint8_t newcommand = 0;   //flag for abandoning current movement

Adafruit_DotStar strip(1, 41, 40, DOTSTAR_BRG); //create dotstar object

TMC2130Stepper driver1 = TMC2130Stepper(S1_EN, S1_dir, S1_step, S1_CS);
TMC2130Stepper driver2 = TMC2130Stepper(S2_EN, S2_dir, S2_step, S2_CS);

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){
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
  
  // IR debug
  // while(1)
  // {
  //   Serial.print(digitalRead(IR_top));
  //   Serial.print(digitalRead(IR_upper));
  //   Serial.print(digitalRead(IR_middle));
  //   Serial.print(digitalRead(IR_lower));
  //   Serial.print(digitalRead(IR_bottom));
  //   Serial.print("-");
  //   Serial.print(analogRead(A1));
  //   Serial.print("-");
  //   Serial.println(analogRead(A0));
  //
  //   delay(250);
  // }
  
  //setup Interrupts
//  attachInterrupt(digitalPinToInterrupt(IR_barrier_rx), IR_barrier_rx_ISR, RISING); //IR stepper side
//  attachInterrupt(digitalPinToInterrupt(IR_barrier_tx), IR_barrier_tx_ISR, RISING);
  
  //Setup LEDs
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  
  //Setup stepper drivers
  driver1.begin(); //initialize pins and registries
	driver1.rms_current(100,0.11,0.4); //set driving current RMS (current RMS, Rsens, hold multiplier) (300max for NEMA 8)
  driver1.stealthChop(1); //enable stealthchopping for quiet runnning
	driver1.microsteps(S1_microsteps); //default 256 for quiet running
  driver1.interpolate(1);
  driver1.double_edge_step(1);

  driver2.begin(); //initialize pins and registries
	driver2.rms_current(25,0.11,0.2); //set driving current RMS (current RMS, Rsens, hold multiplier) (300max for NEMA 8)
  driver2.stealthChop(1); //enable stealthchopping for quiet runnning
	driver2.microsteps(S2_microsteps); //default 256 for quiet running
  driver2.interpolate(1);
  driver2.double_edge_step(1); //step on rising and falling edge
  
  pinMode(S1_step,OUTPUT);
  pinMode(S2_step,OUTPUT);
  
	digitalWrite(S1_EN,LOW); //Enable output on drivers
  digitalWrite(S2_EN,LOW); //Enable output on drivers
  
  //Stepper driver debug
  // Serial.println(driver1.GCONF(), BIN);
  // Serial.println(driver2.GCONF(), BIN);

  //----- calibrate movements --------------------------------------------------
  //move lock
  movelock(open,250,lock_move_steps);   //move the open distance once to make sure it is open before moving the door

  //repeat closing calibration until two consecutive moves report roughly the same number of steps. perfect conditions vary by ~4 steps
  Serial.println("calibrate");
  uint16_t steps_to_close_last = 11;
  while(!(steps_to_close < steps_to_close_last+10) || !(steps_to_close > steps_to_close_last-10)){
    steps_to_close_last = steps_to_close;
    movesimple(up,top,S1_pulsetime,1);      //move door all the way up
    steps_to_close = movesimple(down,bottom,S1_pulsetime,1); //and back down so it starts in closed configuration
    //Serial.println(steps_to_close);
  }
  
  movelock(close,1000,2112);            //move much more than close distance to make sure we are correctly positioned
  movelock(open,1000,32);               //open a bit to prevent coil whine from stepper being under tension from pressing against mount
  
  //----- start I2C on address 0x11 --------------------------------------------
  Wire.begin(0x11);             //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  Wire.onRequest(sendData);     //what to do when being talked to
  Wire.onReceive(receiveEvent); //what to do when/with data received
}

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){
  //------ perform queued move -------------------------------------------------
  if(Q_movesimple[0]){
    S1_busy = 1; //mark module busy as soon as movement command appears
    Q_movesimple[0] = 0; //clear queued move flag
    //copy to new variable so it wont get overwritten by i2c receive ("Queue local")
    uint16_t Ql_movesimple[4] = {0,0,0,0};
    Ql_movesimple[1] = Q_movesimple[1];
    Ql_movesimple[2] = Q_movesimple[2];
    Ql_movesimple[3] = Q_movesimple[3];
    
    //down-movement will always be step-based, up-movement, always feedback based
    steps_last_move = movesimple(Ql_movesimple[1],Ql_movesimple[2],Ql_movesimple[3],!Ql_movesimple[1]); //dir,target,pulsetime,IR based(1) or step(0)
    
    if(!Q_movesimple[0]) S1_busy = 0;}
    
  //------ perform queued vibration of door ------------------------------------
  if(Q_vibrate[0]){
    //Serial.println("Qvib");
    S1_busy = 1; //mark module busy as soon as movement command appears
    Q_vibrate[0] = 0; //clear queued vibrate flag
    uint8_t direction = up; //first move direction
    //Q_vibrate[1];   //steps
    //Q_vibrate[2];   //speed
    //Q_vibrate[3];   //repetitions
    
    for(uint8_t h = 0; h <= (Q_vibrate[3]*2); h++){
      digitalWrite(S1_dir, direction);
      for(uint8_t i = 0; i <= Q_vibrate[1]; i++){
        move(Q_vibrate[2]);}
      direction = !direction;}

    movesimple(down,bottom,S1_pulsetime,1);
    
    S1_busy = 0;}
} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

void movelock(uint8_t direction, uint16_t pulsetime, uint16_t steps){
  if(has_lock){
    digitalWrite(S2_dir, direction); //0 open, 1 close
    
    for(uint16_t i=0; i<steps; i++){
      REG_PORT_OUTSET0 = PORT_PA16; // ~0.4us stepper 1
      delayMicroseconds(pulsetime);
      REG_PORT_OUTCLR0 = PORT_PA16; // ~0.4us
      delayMicroseconds(pulsetime);}}}

//move two microsteps
void move(uint16_t pulsetime){
  REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
  delayMicroseconds(pulsetime);
  REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
  delayMicroseconds(pulsetime);}

//------------------------------------------------------------------------------
//move door to target position, if door is blocked will retry movement
uint16_t movesimple(uint8_t direction, uint8_t target, uint16_t pulsetime, uint8_t IR_movement){
  digitalWrite(S1_dir, direction); //0 open, 1 close
  
  uint8_t temp_target;        //temporary target to coordinate retries
  temp_target = target;
  uint8_t done = 0;           //flag if movement is finsihed
  int16_t steps_counted = 0;  //counts steps for step-based movement
  
  if(IR_movement){ //IR based movement
    while(!done){
      //if(Q_movesimple[0]) done = 1; //stop current operation if new movement command arrives
      
      //read IR states
      IR_state[rx] = digitalRead(IR_barrier_rx);
      IR_state[tx] = digitalRead(IR_barrier_tx);
      IR_state[temp_target] = digitalRead(IR_all[temp_target]);
      
      //if barrier is blocked on moving down, change direction and target to retry target
      if(direction && (IR_state[rx] || IR_state[tx])){
        direction = up;                  //change dir to up
        digitalWrite(S1_dir, direction); //0 up, 1 down
        temp_target = top;}              //retry target is top <-- can be changed for different target e.g. only one step up, or half open
      
      //if we are at the retry target after a retry and not blocked, move back to original target
      if(!(IR_state[temp_target] ^ direction) && (target != temp_target) && !IR_state[rx] && !IR_state[tx]){
        direction = down;                  //change dir to down
        digitalWrite(S1_dir, direction);   //0 up, 1 down
        temp_target = target;}
      
      //if not at defined target, move
      if(IR_state[temp_target] ^ direction){
        move(pulsetime);
        if(direction == up) steps_counted--;
        else steps_counted++;}
      
      //if we have reached our target we are done
      if(!(IR_state[temp_target] ^ direction) && (target == temp_target)) done = 1;}}
  else{ //Step based movement
    while(!done){
      if(Q_movesimple[0]) done = 1; //stop current operation if new movement command
      //read IR states
      IR_state[rx] = digitalRead(IR_barrier_rx);
      IR_state[tx] = digitalRead(IR_barrier_tx);
      
      //if barrier is blocked on moving down, change direction
      if(direction && (IR_state[rx] || IR_state[tx])){
        direction = up;                    //change dir to up
        digitalWrite(S1_dir, direction);}  //0 up, 1 down
      
      //move all the way to the top if IR is blocked
      //if((steps_counted <= 0) && !IR_state[rx] && !IR_state[tx])
      //if we are moving up or we have reached the top, change direction to down but only if IR is not blocked
      if(((direction == up) || (steps_counted <= 0)) && (!IR_state[rx] && !IR_state[tx])){
        direction = down;                 //change dir to down
        digitalWrite(S1_dir, direction);} //0 up, 1 down
      
      //move up if blocked and down only if not blocked
      if((direction == down) || ((direction == up) && !(steps_counted <= 0))){
        if(direction == up){ //when moving up, move up slower
          steps_counted--;
          move(5000);}       //very slow up movement
        else{                //down movement at normal speed
          steps_counted++;
          move(pulsetime);}}
      
      //if we made the number of calibrated steps for down movement, we are done
      if(steps_counted >= steps_to_close + 5) done = 1;}} //add a few more steps to counter drift
  
  //done with movement
  return steps_counted;}

//------------------------------------------------------------------------------
//close step by step with timing and feedback
//calibrate necessary, uses Irs parallel to tube for feedback move
void closefancy(uint8_t start, uint8_t stop, uint16_t pulsetime){
  uint8_t closing;
  uint8_t lower_door = 1;
  uint32_t move_time;

  //move through all IR barrier intervals step by step
  for(int i = start; i < stop; i++){
    //close step by step with feedback
    closing = 1;
    while(closing){
      //lower door from start to stop
      digitalWrite(S1_dir, down); //move down
      move_time = millis();
      while((digitalRead(IR_all[i+1]) == 0) && lower_door){
        if((millis() - move_time) < (move_interval_times[i] * 1.2)){
          move(pulsetime);}
        else lower_door = 0;}}

      //move up to reset if door doesn't reach target within time
      //Problem: If door is already above sensor, it will start moving down again, potentially
      //unravelling the string
      //solution 1: always move all the way up
      //solution 2: use movement timing <- now
      digitalWrite(S1_dir, up); //move up
      move_time = millis();
      //even if IR is already unblocked (mouse lifts door up) move back a bit to prevent string from uncurling
      while(((digitalRead(IR_all[i]) == 1) || ((millis() - move_time) < (move_interval_times[i] * 1.1))) && !lower_door){
        move(pulsetime);}
      //if lower_door true, move finished, else door moved up, try again
      if(lower_door) closing = 0;
      lower_door = 1;}}

//------------------------------------------------------------------------------
//Do a calibration movement where we time how long it takes the door to reach each IR barrier point
void calibrate(uint16_t pulsetime){
  movesimple(up, top, pulsetime,1);
  //now move down
  digitalWrite(S1_dir, down);
  uint32_t move_time = millis();
  while(digitalRead(IR_upper) == 0){
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);}
  move_interval_times[0] = millis() - move_time;
  delay(100);

  move_time = millis();
  while(digitalRead(IR_middle) == 0){
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);}
  move_interval_times[1] = millis() - move_time;
  delay(100);
  
  move_time = millis();
  while(digitalRead(IR_lower) == 0){
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);}
  move_interval_times[2] = millis() - move_time;
  delay(100);
  
  move_time = millis();
  while(digitalRead(IR_bottom) == 0){
    REG_PORT_OUTSET0 = PORT_PA08; // ~0.4us stepper 1
    delayMicroseconds(pulsetime);
    REG_PORT_OUTCLR0 = PORT_PA08; // ~0.4us
    delayMicroseconds(pulsetime);}
  move_interval_times[3] = millis() - move_time;
  delay(100);
  
  movesimple(up, top, pulsetime,1);
  
  // Serial.print(move_interval_times[0]);
  // Serial.print(" ");
  // Serial.print(move_interval_times[1]);
  // Serial.print(" ");
  // Serial.print(move_interval_times[2]);
  // Serial.print(" ");
  // Serial.println(move_interval_times[3]);
}

//------------------------------------------------------------------------------
//I2C receive instructions
void receiveEvent(int bytes_incoming){
  uint8_t inputbuffer[7] = {0,0,0,0,0,0,0};
  uint8_t option;
  
  //collect bytes from I2C
  for(uint8_t i = 0; i < bytes_incoming; i++){
    uint8_t c = Wire.read();
    inputbuffer[i] = c;}
  
  //----- process received data -----
  option = inputbuffer[0];
  
  if(option == 0){ //set config
    //write config for various door functions
    //driver 1 rms
    //driver 1 holdcurrent
    //driver 1 speed
    //driver 2 rms
    //driver 2 holdcurrent
    //driver 2 speed
  }
  
  if(option == 1){ //calibrate
    S1_pulsetime = S1_pulsetime | inputbuffer[2];
    S1_pulsetime = (S1_pulsetime << 8) | inputbuffer[1];
    
    calibrate(S1_pulsetime);}
  
  if(option == 2){ //move door with simple feedback
    Q_movesimple[1] = inputbuffer[1];                           //direction
    Q_movesimple[2] = inputbuffer[2];                           //target
    Q_movesimple[3] = 0;                                       //clear, because bit-shifting
    Q_movesimple[3] = Q_movesimple[3] | inputbuffer[4];         //pulsetime
    Q_movesimple[3] = (Q_movesimple[3] << 8) | inputbuffer[3];  //pulsetime
    
    Q_movesimple[0] = 1; //queues movement
    newcommand = 1;}
  
  if(option == 3){ //close door with fancy stepwise feedback
    uint8_t start = inputbuffer[1];
    uint8_t stop = inputbuffer[2];
    
    closefancy(start, stop, S1_pulsetime);}
  
  if(option == 4){ //door feedback vibration
    Q_vibrate[1] = inputbuffer[1];  //steps
    Q_vibrate[2] = inputbuffer[2];  //speed
    Q_vibrate[3] = inputbuffer[3];  //repetitions
    Q_vibrate[0] = 1;}}            //queues movement

//I2C send instructions
void sendData(){
  uint8_t sendbuffer[2] = {0,0}; //buffer for sending data over I2C
  //potentially send different datapacks
  //  if(selectdata == x)
  //  {sendbuffer1 || sendbuffer2}
  
  //sendbuffer[0] IR status
  sendbuffer[0] = sendbuffer[0] | ((IR_state[top] & 0x01) << 0);
  sendbuffer[0] = sendbuffer[0] | ((IR_state[upper] & 0x01) << 1);
  sendbuffer[0] = sendbuffer[0] | ((IR_state[middle] & 0x01) << 2);
  sendbuffer[0] = sendbuffer[0] | ((IR_state[lower] & 0x01) << 3);
  sendbuffer[0] = sendbuffer[0] | ((IR_state[bottom] & 0x01) << 4);
  sendbuffer[0] = sendbuffer[0] | ((IR_state[rx] & 0x01) << 5);
  sendbuffer[0] = sendbuffer[0] | ((IR_state[tx] & 0x01) << 6);
  //sendbuffer[0] = sendbuffer[0] | ((? & 0x01) << 7);
  
  sendbuffer[1] = sendbuffer[1] | ((S1_busy & 0x01) << 0);
  
  Wire.write(sendbuffer,2);}
