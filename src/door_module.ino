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

Side View              Front View
*///----------------------------------------------------------------------------

#include <i2c_t3.h>
#include <TMCStepper.h>

//Stepper 1
const uint8_t S1_step	=	22;
const uint8_t S1_dir	=	3;
const uint8_t S1_EN		=	23;
const uint8_t S1_CS		=	2;
const uint8_t S1_microsteps = 32;

volatile uint8_t S1_move = 0; //flag to make stepper move
uint32_t S1_timer = 0;        //time to the next step pulse
uint8_t S1_target;            //IR barrier target (top or bottom)
uint8_t S1_temp_target;       //temporary target to coordinate retries
int32_t S1_steps_counted = 0; //counts steps for step-based movement
uint8_t S1_movetype;          //IR based or step based movement
uint8_t S1_direction;         //movement direction, up or down
uint16_t S1_pulsetime = 250;  //global speed, preset for calibration movement
int16_t S1_steps_to_close;    //holds the number of steps needed to go from open to close position of the door

//Stepper 2
const uint8_t S2_step	=	8;
const uint8_t S2_dir	=	10;
const uint8_t S2_EN		=	9;
const uint8_t S2_CS		=	7;
const uint8_t S2_microsteps = 32;

volatile uint8_t S2_move = 0; //flag to make stepper move
uint32_t S2_timer = 0;        //time to the next step pulse
uint8_t S2_target;            //IR barrier target (top or bottom)
uint8_t S2_temp_target;       //temporary target to coordinate retries
int32_t S2_steps_counted = 0; //counts steps for step-based movement
uint8_t S2_movetype;          //IR based or step based movement
uint8_t S2_direction;         //movement direction, up or down
uint16_t S2_pulsetime = 250;  //global speed, preset for calibration movement
int16_t S2_steps_to_close;    //holds the number of steps needed to go from open to close position of the door

//door direction
const uint8_t up = 0;
const uint8_t down = 1;

//IR Sensor Pins (HIGH = Blocked) Order: tx1,tx2,rx1,rx2,top,bottom
const int IR1_all[6] = {1,0,21,24,25,4};
const int IR2_all[6] = {6,5,20,15,16,17};
volatile uint8_t IR1_state[6] = {0,0,0,0,0,0}; //array with current state of all IR barriers
volatile uint8_t IR2_state[6] = {0,0,0,0,0,0}; //array with current state of all IR barriers

//to improve function readability - mapped to IRx_state
const uint8_t tx1 = 0;
const uint8_t tx2 = 1;
const uint8_t rx1 = 2;
const uint8_t rx2 = 3;
const uint8_t top = 4;
const uint8_t bottom = 5;

//LEDs
const uint8_t LED1 = 14;
const uint8_t LED2 = 26;

//global variable to move the blocking stepper movement out of the i2c function
volatile uint16_t Q_movesimple[4]; //queue a move command to release I2C bus

TMC5160Stepper driver1(S1_CS, 0.075f, 11, 12, 13); //(CS, Rsens, MOSI, MISO, SCK)
TMC5160Stepper driver2(S2_CS, 0.075f, 11, 12, 13);

//debugging
uint32_t debugtimer;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){
  //while(!Serial); //wait for serial connection
  
  //Setup IR Sensors (high when interrrupted)
  for(uint8_t i = 0;i < 6;i ++){
    pinMode(IR1_all[i],INPUT);
    pinMode(IR2_all[i],INPUT);
  }

  //Stepper pins
  pinMode(S1_step,OUTPUT);
  pinMode(S1_dir,OUTPUT);
  pinMode(S1_EN,OUTPUT);
  pinMode(S1_CS,OUTPUT);

  pinMode(S2_step,OUTPUT);
  pinMode(S2_dir,OUTPUT);
  pinMode(S2_EN,OUTPUT);
  pinMode(S2_CS,OUTPUT);

  //Setup LEDs
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  
  //Setup stepper drivers
  driver1.begin();                    //initialize pins and registries
	driver1.rms_current(150,0.4);       //set driving current (current RMS, hold multiplier) (300max for NEMA 8)
  driver1.en_pwm_mode(true);          //enable stealthchopping for quiet runnning
	driver1.microsteps(S1_microsteps);  //set number of microsteps max 256
  driver1.pwm_autoscale(true);        //needed for stealthChop
  driver1.dedge(true);                //step on both edges of the signal

  driver2.begin();                    //initialize pins and registries
	driver2.rms_current(150,0.4);       //set driving current (current RMS, hold multiplier) (300max for NEMA 8)
  driver2.en_pwm_mode(true);          //enable stealthchopping for quiet runnning
	driver2.microsteps(S2_microsteps);  //set number of microsteps max 256
  driver2.pwm_autoscale(true);        //needed for stealthChop
  driver2.dedge(true);                //step on both edges of the signal
  
	digitalWrite(S1_EN,LOW);  //Enable output on drivers
  digitalWrite(S2_EN,LOW);  //Enable output on drivers
  
  //Stepper driver debug, print config values
  // Serial.println(driver1.GCONF(), BIN);
  // Serial.println(driver2.GCONF(), BIN);

  //----- start I2C on address 0x10 --------------------------------------------
  Wire.begin(0x10);             //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  Wire.onRequest(sendData);     //what to do when being talked to
  Wire.onReceive(receiveEvent); //what to do when/with data received

  //----- calibrate movements --------------------------------------------------
  //repeat closing calibration until two consecutive moves report roughly the same number of steps. perfect conditions vary by ~4 steps
  S1_move = 1;
  S2_move = 1;
  //Stepper 1
  Serial.println("calibrate S1");
  uint16_t steps_to_close_last = 21;
  while(!(S1_steps_to_close < steps_to_close_last+20) || !(S1_steps_to_close > steps_to_close_last-20)){
    steps_to_close_last = S1_steps_to_close;
    calibrate(0,up,S1_pulsetime);       //move door all the way up
    S1_steps_to_close = calibrate(0,down,S1_pulsetime); //and back down so it starts in closed configuration
    S1_steps_counted = S1_steps_to_close; //since we start at the bottom, steps counted should be the maximum
    Serial.println(S1_steps_to_close);
  }
  //Stepper 2
  Serial.println("calibrate S2");
  steps_to_close_last = 21;
  while(!(S2_steps_to_close < steps_to_close_last+20) || !(S2_steps_to_close > steps_to_close_last-20)){
    steps_to_close_last = S2_steps_to_close;
    calibrate(1,up,S2_pulsetime);       //move door all the way up
    S2_steps_to_close = calibrate(1,down,S1_pulsetime); //and back down so it starts in closed configuration
    S2_steps_counted = S2_steps_to_close; //since we start at the bottom, steps counted should be the maximum
    Serial.println(S2_steps_to_close);
  }
  S1_move = 0;
  S2_move = 0;
}

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){
  //debugging
  // if(millis() - debugtimer >= 500){
  //   debugtimer = millis();
  //   Serial.print(S1_steps_counted);
  //   Serial.print(" | ");
  //   Serial.println(S2_steps_counted);
  // }

  //read IR barrier status
  for(uint8_t i = 0;i < 6;i ++){
    IR1_state[i] = digitalRead(IR1_all[i]);
    IR2_state[i] = digitalRead(IR2_all[i]);
  }

  //light up LEDs when stepper busy
  if(S1_move) digitalWrite(LED1,HIGH);
  if(!S1_move) digitalWrite(LED1,LOW);
  if(S2_move) digitalWrite(LED2,HIGH);
  if(!S2_move) digitalWrite(LED2,LOW);

  //if new movement command arrives copy to stepper specific command array
  if(Q_movesimple[0]){ 
    Q_movesimple[0] = 0; //clear new command flag

    if(Q_movesimple[3] == 0){ //stepper 1
      //process movement command
      S1_move = 1;  //flag to start moving
      S1_direction = Q_movesimple[1]; //up = 0, down = 1
      S1_pulsetime = Q_movesimple[2];
      S1_movetype = !Q_movesimple[1]; //up movement always IR based, down always step based

      if(S1_direction == up) S1_target = top;
      if(S1_direction == down) S1_target = bottom;
      S1_temp_target = S1_target;
      digitalWrite(S1_dir, S1_direction); //0 open, 1 close
    }

    if(Q_movesimple[3] == 1){ //stepper 2
      //process movement command
      S2_move = 1;  //flag to start moving
      S2_direction = Q_movesimple[1]; //up = 0, down = 1
      S2_pulsetime = Q_movesimple[2];
      S2_movetype = !Q_movesimple[1]; //up movement always IR based 1, down always step based 0
      
      if(S2_direction == up) S2_target = top;
      if(S2_direction == down) S2_target = bottom;
      S2_temp_target = S2_target;
      digitalWrite(S2_dir, S2_direction); //0 open, 1 close
    }
  }

  //--------------------------------------------------------------------------------------------------
  //----- stepper 1 ----------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------------
  if(S1_move){
    if(S1_movetype == 1){ //IR based movement
      //if barrier is blocked on moving down, change direction and target to retry target
      if(S1_direction && (IR1_state[tx1] || IR1_state[tx2] || IR1_state[rx1] || IR1_state[rx2])){
        S1_direction = up;                  //change dir to up
        digitalWrite(S1_dir, S1_direction); //0 up, 1 down
        S1_temp_target = top; //retry target is top
      }
      //if we are at the retry target after a retry and not blocked, move back to original target
      if(!(IR1_state[S1_temp_target] ^ S1_direction) && (S1_target != S1_temp_target) && (!IR1_state[tx1] && !IR1_state[tx2] && !IR1_state[rx1] && !IR1_state[rx2])){
        S1_direction = down;                  //change dir to down
        digitalWrite(S1_dir, S1_direction);   //0 up, 1 down
        S1_temp_target = S1_target;
      }
      //if not at defined target, move
      if(IR1_state[S1_temp_target] ^ S1_direction){
        if(micros() - S1_timer >= S1_pulsetime){
          S1_timer = micros(); //update timer
          digitalToggleFast(S1_step); //make a step
        }
      }
      //if we have reached our target we are done
      if(!(IR1_state[S1_temp_target] ^ S1_direction) && (S1_target == S1_temp_target)){
        S1_move = 0;
        if(S1_target == top) S1_steps_counted = 0;                    //reset step counter,0 at top
        if(S1_target == bottom) S1_steps_counted = S1_steps_to_close; //reset step counter, calibrated value at bottom
      }
    }
    if(S1_movetype == 0){ //Step based movement
      //if barrier is blocked on moving down, change direction
      if(S1_direction && (IR1_state[tx1] || IR1_state[tx2] || IR1_state[rx1] || IR1_state[rx2])){
        S1_direction = up;                    //change dir to up
        digitalWrite(S1_dir, S1_direction);  //0 up, 1 down
      }
      //if we are moving up or we have reached the top, change direction to down but only if IR is not blocked
      if(((S1_direction == up) || (S1_steps_counted <= 0)) && (!IR1_state[tx1] && !IR1_state[tx2] && !IR1_state[rx1] && !IR1_state[rx2])){
        S1_direction = down;                 //change dir to down
        digitalWrite(S1_dir, S1_direction); //0 up, 1 down
      }
      //move up if blocked and down only if not blocked
      if((S1_direction == down) || ((S1_direction == up) && !(S1_steps_counted <= 0))){
        if(S1_direction == up){ //when moving up, move up slower
          if(micros() - S1_timer >= 10000){
            S1_timer = micros(); //update timer
            digitalToggleFast(S1_step); //make a step
            S1_steps_counted--;
          }
        }
        else{                //down movement at normal speed
          if(micros() - S1_timer >= S1_pulsetime){
            S1_timer = micros(); //update timer
            digitalToggleFast(S1_step); //make a step
            S1_steps_counted++;
          }
        }
      }
      //if we made the number of calibrated steps for down movement, we are done
      if(S1_steps_counted >= S1_steps_to_close + 5) S1_move = 0; //add a few more steps to counter drift
    }
  }

  //--------------------------------------------------------------------------------------------------
  //----- stepper 2 ----------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------------
  if(S2_move){
    if(S2_movetype == 1){ //IR based movement
      //if barrier is blocked on moving down, change direction and target to retry target
      if(S2_direction && (IR2_state[tx1] || IR2_state[tx2] || IR2_state[rx1] || IR2_state[rx2])){
        S2_direction = up;                  //change dir to up
        digitalWrite(S2_dir, S2_direction); //0 up, 1 down
        S2_temp_target = top; //retry target is top
      }
      //if we are at the retry target after a retry and not blocked, move back to original target
      if(!(IR2_state[S2_temp_target] ^ S2_direction) && (S2_target != S2_temp_target) && (!IR2_state[tx1] && !IR2_state[tx2] && !IR2_state[rx1] && !IR2_state[rx2])){
        S2_direction = down;                  //change dir to down
        digitalWrite(S2_dir, S2_direction);   //0 up, 1 down
        S2_temp_target = S2_target;
      }
      //if not at defined target, move
      if(IR2_state[S2_temp_target] ^ S2_direction){
        if(micros() - S2_timer >= S2_pulsetime){
          S2_timer = micros(); //update timer
          digitalToggleFast(S2_step); //make a step
        }
      }
      //if we have reached our target we are done
      if(!(IR2_state[S2_temp_target] ^ S2_direction) && (S2_target == S2_temp_target)){
        S2_move = 0;
        if(S2_target == top) S2_steps_counted = 0;                    //reset step counter,0 at top
        if(S2_target == bottom) S2_steps_counted = S2_steps_to_close; //reset step counter, calibrated value at bottom
      }
    }
    if(S2_movetype == 0){ //Step based movement
      //if barrier is blocked on moving down, change direction
      if(S2_direction && (IR2_state[tx1] || IR2_state[tx2] || IR2_state[rx1] || IR2_state[rx2])){
        S2_direction = up;                    //change dir to up
        digitalWrite(S2_dir, S2_direction);  //0 up, 1 down
      }
      //if we are moving up or we have reached the top, change direction to down but only if IR is not blocked
      if(((S2_direction == up) || (S2_steps_counted <= 0)) && (!IR2_state[tx1] && !IR2_state[tx2] && !IR2_state[rx1] && !IR2_state[rx2])){
        S2_direction = down;                 //change dir to down
        digitalWrite(S2_dir, S2_direction); //0 up, 1 down
      }
      //move up if blocked and down only if not blocked
      if((S2_direction == down) || ((S2_direction == up) && !(S2_steps_counted <= 0))){
        if(S2_direction == up){ //when moving up, move up slower
          if(micros() - S2_timer >= 10000){            
            S2_timer = micros(); //update timer
            digitalToggleFast(S2_step); //make a step
            S2_steps_counted--;
          }
        }
        else{                //down movement at normal speed
          if(micros() - S2_timer >= S2_pulsetime){
            S2_timer = micros(); //update timer
            digitalToggleFast(S2_step); //make a step
            S2_steps_counted++;
          }
        }
      }
      //if we made the number of calibrated steps for down movement, we are done
      if(S2_steps_counted >= S2_steps_to_close + 5) S2_move = 0; //add a few more steps to counter drift
    }
  }
} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//move one microstep -----------------------------------------------------------
void move(uint8_t stepper, uint16_t pulsetime){ 
  if(stepper == 1){
    digitalToggleFast(S1_step);
    delayMicroseconds(pulsetime);
  }
  if(stepper == 2){
    digitalToggleFast(S2_step);
    delayMicroseconds(pulsetime);
  }
}

//record number of steps needed to close/open door -----------------------------
uint16_t calibrate(uint8_t stepper,uint8_t direction, uint16_t pulsetime){
  uint8_t target;             //IR barrier target (top or bottom)
  uint8_t temp_target;        //temporary target to coordinate retries
  uint8_t done = 0;           //flag if movement is finished
  int16_t steps_counted = 0;  //counts steps for step-based movement

  if(direction == up) target = top;
  if(direction == down) target = bottom;

  //stepper 1
  if(stepper == 0){
    temp_target = target;
    digitalWrite(S1_dir, direction); //0 open, 1 close

    while(!done){
      //read IR sensors
      IR1_state[tx1] = digitalRead(IR1_all[tx1]);
      IR1_state[tx2] = digitalRead(IR1_all[tx2]);
      IR1_state[rx1] = digitalRead(IR1_all[rx1]);
      IR1_state[rx2] = digitalRead(IR1_all[rx2]);
      IR1_state[temp_target] = digitalRead(IR1_all[temp_target]);
      
      //if barrier is blocked on moving down, change direction and target to retry target
      if(direction && (IR1_state[tx1] || IR1_state[tx2] || IR1_state[rx1] || IR1_state[rx2])){
        direction = up;                  //change dir to up
        digitalWrite(S1_dir, direction); //0 up, 1 down
        temp_target = top; //retry target is top <-- can be changed for different target e.g. only one step up, or half open
      }
      //if we are at the retry target after a retry and not blocked, move back to original target
      if(!(IR1_state[temp_target] ^ direction) && (target != temp_target) && (!IR1_state[tx1] && !IR1_state[tx2] && !IR1_state[rx1] && !IR1_state[rx2])){
        direction = down;                  //change dir to down
        digitalWrite(S1_dir, direction);   //0 up, 1 down
        temp_target = target;
      }
      //if not at defined target, move
      if(IR1_state[temp_target] ^ direction){
        move(1,pulsetime);
        if(direction == up) steps_counted--;
        else steps_counted++;
      }
      //if we have reached our target we are done
      if(!(IR1_state[temp_target] ^ direction) && (target == temp_target)) done = 1;
    }
  }
  //stepper 2
  if(stepper == 1){
    temp_target = target;
    done = 0; 
    steps_counted = 0;

    digitalWrite(S2_dir, direction); //0 open, 1 close

    while(!done){
      //read IR sensors
      IR2_state[tx1] = digitalRead(IR2_all[tx1]);
      IR2_state[tx2] = digitalRead(IR2_all[tx2]);
      IR2_state[rx1] = digitalRead(IR2_all[rx1]);
      IR2_state[rx2] = digitalRead(IR2_all[rx2]);
      IR2_state[temp_target] = digitalRead(IR2_all[temp_target]);
      
      //if barrier is blocked on moving down, change direction and target to retry target
      if(direction && (IR2_state[tx1] || IR2_state[tx2] || IR2_state[rx1] || IR2_state[rx2])){
        direction = up;                  //change dir to up
        digitalWrite(S2_dir, direction); //0 up, 1 down
        temp_target = top; //retry target is top <-- can be changed for different target e.g. only one step up, or half open
      }
      //if we are at the retry target after a retry and not blocked, move back to original target
      if(!(IR2_state[temp_target] ^ direction) && (target != temp_target) && (!IR2_state[tx1] && !IR2_state[tx2] && !IR2_state[rx1] && !IR2_state[rx2])){
        direction = down;                  //change dir to down
        digitalWrite(S2_dir, direction);   //0 up, 1 down
        temp_target = target;
      }
      //if not at defined target, move
      if(IR2_state[temp_target] ^ direction){
        move(2,pulsetime);
        if(direction == up) steps_counted--;
        else steps_counted++;
      }
      //if we have reached our target we are done
      if(!(IR2_state[temp_target] ^ direction) && (target == temp_target)) done = 1;
    }
  }
  //done with movement
  return steps_counted;
}

//I2C receive instructions -----------------------------------------------------
void receiveEvent(size_t bytes_incoming){
  uint8_t rcv[7] = {0,0,0,0,0,0,0};
  uint8_t option;
  
  //collect bytes from I2C
  for(uint8_t i = 0; i < bytes_incoming; i++){
    uint8_t c = Wire.read();
    rcv[i] = c;
  }
  //process received data
  option = rcv[0];
  
  if(option == 0){ //set config
    //write config for various door functions
    //driver 1 rms
    //driver 1 holdcurrent
    //driver 1 speed
    //driver 2 rms
    //driver 2 holdcurrent
    //driver 2 speed
  }
  if(option == 1){}
  if(option == 2){ //move door with simple feedback
    Q_movesimple[1] = rcv[1];                           //direction
    Q_movesimple[2] = 0;                                //clear, because bit-shifting
    Q_movesimple[2] = Q_movesimple[2] | rcv[3];         //pulsetime
    Q_movesimple[2] = (Q_movesimple[2] << 8) | rcv[2];  //pulsetime
    Q_movesimple[3] = rcv[4];                           //which door

    Q_movesimple[0] = 1; //queues movement
  }
  if(option == 3){}
  if(option == 4){}
}            

//I2C send instructions --------------------------------------------------------
void sendData(){
  uint8_t sendbuffer[2] = {0,0}; //buffer for sending data over I2C
  
  sendbuffer[0] = sendbuffer[0] | (((S1_move & 0x01) & 0x01) << 0);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[tx1] & 0x01) << 1);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[tx2] & 0x01) << 2);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[rx1] & 0x01) << 3);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[rx2] & 0x01) << 4);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[top] & 0x01) << 5);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[bottom] & 0x01) << 6);
  //sendbuffer[0] = sendbuffer[0] | ((? & 0x01) << 7);

  sendbuffer[1] = sendbuffer[1] | (((S2_move & 0x01) & 0x01) << 0);
  sendbuffer[1] = sendbuffer[1] | ((IR2_state[tx1] & 0x01) << 1);
  sendbuffer[1] = sendbuffer[1] | ((IR2_state[tx2] & 0x01) << 2);
  sendbuffer[1] = sendbuffer[1] | ((IR2_state[rx1] & 0x01) << 3);
  sendbuffer[1] = sendbuffer[1] | ((IR2_state[rx2] & 0x01) << 4);
  sendbuffer[1] = sendbuffer[1] | ((IR2_state[top] & 0x01) << 5);
  sendbuffer[1] = sendbuffer[1] | ((IR2_state[bottom] & 0x01) << 6);
  //sendbuffer[1] = sendbuffer[1] | ((? & 0x01) << 7);
  
  Wire.write(sendbuffer,2);
}
