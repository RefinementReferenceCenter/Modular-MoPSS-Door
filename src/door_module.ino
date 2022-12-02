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

#include <i2c_t3.h>
#include <TMCStepper.h>

//Stepper 1
const uint8_t S1_dir	=	3;
const uint8_t S1_step	=	22; //port PA08
const uint8_t S1_EN		=	23;
const uint8_t S1_CS		=	2;
const uint8_t S1_microsteps = 32;

volatile uint8_t S1_busy = 0; //Is stepper moving
uint16_t S1_pulsetime = 250;  //global speed, for calibration movement
int16_t steps_to_close;       //holds the number of steps needed to go from open to close position of the door
int16_t steps_last_move;      //holds current position in steps when move is interrupted

uint8_t S1_move = 0;      //flag to make stepper move
uint32_t S1_timer = 0;        //time to the next step pulse
uint16_t S1_next_pulse = 0;  //how long to wait for the next stepping pulse

//Stepper 2
const uint8_t S2_dir	=	10;
const uint8_t S2_step	=	8; //port PA16
const uint8_t S2_EN		=	9;
const uint8_t S2_CS		=	7;
const uint8_t S2_microsteps = 32;

volatile uint8_t S2_busy = 0;      //Is stepper moving

uint32_t S2_timer = 0;      //time to the next step pulse
uint16_t S2_next_pulse = 0; //how long to wait for the next stepping pulse
uint8_t S2_move = 0;      //flag to make stepper move

//IR Sensor Pins (HIGH = Blocked)
const int IRtx1[2] = {1,0};
const int IRrx1[2] = {21,24};
const int IRd1[2] = {25,4};  //top, bottom

const int IRtx2[2] = {6,5};
const int IRrx2[2] = {20,15};
const int IRd2[2] = {16,17};  //top, bottom

//Order: tx1,tx2,rx1,rx2,top, bottom
const int IR1_all[6] = {IRtx1[0],IRtx1[1],IRrx1[0],IRrx1[1],IRd1[0],IRd1[1]};
const int IR2_all[6] = {IRtx2[0],IRtx2[1],IRrx2[0],IRrx2[1],IRd2[0],IRd2[1]};
volatile uint8_t IR1_state[6] = {0,0,0,0,0,0}; //array with current state of all IR barriers
volatile uint8_t IR2_state[6] = {0,0,0,0,0,0}; //array with current state of all IR barriers

//to improve function readability - mapped to IRx_state
const uint8_t tx1 = 0;
const uint8_t tx2 = 1;
const uint8_t rx1 = 2;
const uint8_t rx2 = 3;
const uint8_t top = 4;    //IR LEDs
const uint8_t bottom = 5;

//door direction
const uint8_t up = 0;
const uint8_t down = 1;

//LEDs
const uint8_t LED1 = 14;
const uint8_t LED2 = 26;

uint8_t closedoor = 1;

uint32_t move_interval_times[4];

//global variable to move the blocking stepper movement out of the i2c function
volatile uint16_t Q_movesimple[4]; //queue a movesimple command to release I2C bus
volatile uint16_t Q_vibrate[4];
volatile uint8_t newcommand = 0;   //flag for abandoning current movement

TMC5160Stepper driver1(S1_CS, 0.075f, 11, 12, 13); //(CS, Rsens, MOSI, MISO, SCK)
TMC5160Stepper driver2(S2_CS, 0.075f, 11, 12, 13);

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){
  //while(!Serial); //wait for serial connection
  
  //Setup IR Sensors (high when interrrupted)
  pinMode(IRtx1[0],INPUT);
  pinMode(IRtx1[1],INPUT);
  pinMode(IRrx1[0],INPUT);
  pinMode(IRrx1[1],INPUT);
  pinMode(IRd1[0],INPUT);
  pinMode(IRd1[1],INPUT);

  pinMode(IRtx2[0],INPUT);
  pinMode(IRtx2[1],INPUT);
  pinMode(IRrx2[0],INPUT);
  pinMode(IRrx2[1],INPUT);
  pinMode(IRd2[0],INPUT);
  pinMode(IRd2[1],INPUT);

  //Stepper pins
  pinMode(S1_step,OUTPUT);
  pinMode(S1_dir,OUTPUT);
  pinMode(S1_EN,OUTPUT);
  pinMode(S1_CS,OUTPUT);

  pinMode(S2_step,OUTPUT);
  pinMode(S2_dir,OUTPUT);
  pinMode(S2_EN,OUTPUT);
  pinMode(S2_CS,OUTPUT);


  //setup Interrupts
//  attachInterrupt(digitalPinToInterrupt(IR_barrier_rx), IR_barrier_rx_ISR, RISING); //IR stepper side
//  attachInterrupt(digitalPinToInterrupt(IR_barrier_tx), IR_barrier_tx_ISR, RISING);
  
  //Setup LEDs
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  
  //Setup stepper drivers
  driver1.begin(); //initialize pins and registries
	driver1.rms_current(100,0.4); //set driving current RMS (current RMS, hold multiplier) (300max for NEMA 8)
  driver1.en_pwm_mode(true);  //enable stealthchopping for quiet runnning
	driver1.microsteps(S1_microsteps); //default 256 for quiet running
  driver1.pwm_autoscale(true);     // Needed for stealthChop
  driver1.dedge(true);  //step on both edges of the signal

  driver2.begin(); //initialize pins and registries
	driver2.rms_current(100,0.4); //set driving current RMS (current RMS, hold multiplier) (300max for NEMA 8)
  driver2.en_pwm_mode(true);  //enable stealthchopping for quiet runnning
	driver2.microsteps(S2_microsteps); //default 256 for quiet running
  driver2.pwm_autoscale(true);     // Needed for stealthChop
  driver2.dedge(true);  //step on both edges of the signal
  
	digitalWrite(S1_EN,LOW); //Enable output on drivers
  digitalWrite(S2_EN,LOW); //Enable output on drivers
  
  //Stepper driver debug
  // Serial.println(driver1.GCONF(), BIN);
  // Serial.println(driver2.GCONF(), BIN);

  //----- calibrate movements --------------------------------------------------
  
  //repeat closing calibration until two consecutive moves report roughly the same number of steps. perfect conditions vary by ~4 steps
  Serial.println("calibrate");
  uint16_t steps_to_close_last = 11;
  while(!(steps_to_close < steps_to_close_last+10) || !(steps_to_close > steps_to_close_last-10)){
    steps_to_close_last = steps_to_close;
    movesimple(up,top,S1_pulsetime,1);      //move door all the way up
    steps_to_close = movesimple(down,bottom,S1_pulsetime,1); //and back down so it starts in closed configuration
    //Serial.println(steps_to_close);
  }
  
  //----- start I2C on address 0x10 --------------------------------------------
  //Wire.begin(0x10);             //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //Wire.onRequest(sendData);     //what to do when being talked to
  //Wire.onReceive(receiveEvent); //what to do when/with data received

  S1_timer = micros();
  S2_timer = micros();
}

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){


  //move the steppers
  // if(S1_move && (micros() - S1_timer >= S1_next_pulse)){
  //   digitalToggleFast(S1_step);
  //   S1_timer = micros();
  // }

  // if(S2_move && (micros() - S2_timer >= S2_next_pulse)){
  //   digitalToggleFast(S2_step);
  //   S2_timer = micros();
  // }


  //Debug LEDs
  // Serial.print(digitalRead(IRtx1[0]));
  // Serial.print("-");
  // Serial.print(digitalRead(IRtx1[1]));
  // Serial.print("-");
  // Serial.print(digitalRead(IRrx1[0]));
  // Serial.print("-");
  // Serial.print(digitalRead(IRrx1[1]));
  // Serial.print("-");
  // Serial.print(digitalRead(IRd1[0]));
  // Serial.print("-");
  // Serial.print(digitalRead(IRd1[1]));
  // Serial.println();
  // delay(250);

  //debug movement
  movesimple(up,top,250,1);
  delay(5000);
  movesimple(down,bottom,250,1);
  delay(5000);



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
    
    if(!Q_movesimple[0]) S1_busy = 0;
  }
    
  // //------ perform queued vibration of door ------------------------------------
  // if(Q_vibrate[0]){
  //   //Serial.println("Qvib");
  //   Q_vibrate[0] = 0; //clear queued vibrate flag
  //   S1_busy = 0;
  // }
} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//move two microsteps
void move(uint8_t stepper, uint16_t pulsetime){ 
  if(stepper == 1){
    digitalWriteFast(S1_step,HIGH);
    delayMicroseconds(pulsetime);
    digitalWriteFast(S1_step,LOW);
    delayMicroseconds(pulsetime);
  }
  if(stepper == 2){
    digitalWriteFast(S2_step,HIGH);
    delayMicroseconds(pulsetime);
    digitalWriteFast(S2_step,LOW);
    delayMicroseconds(pulsetime);
  }
}

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
  else{ //Step based movement
    while(!done){
      if(Q_movesimple[0]) done = 1; //stop current operation if new movement command
      //read IR states
      //read IR sensors
      IR1_state[tx1] = digitalRead(IR1_all[tx1]);
      IR1_state[tx2] = digitalRead(IR1_all[tx2]);
      IR1_state[rx1] = digitalRead(IR1_all[rx1]);
      IR1_state[rx2] = digitalRead(IR1_all[rx2]);
      
      //if barrier is blocked on moving down, change direction
      if(direction && (IR1_state[tx1] || IR1_state[tx2] || IR1_state[rx1] || IR1_state[rx2])){
        direction = up;                    //change dir to up
        digitalWrite(S1_dir, direction);  //0 up, 1 down
      }
      //move all the way to the top if IR is blocked
      //if((steps_counted <= 0) && !IR_state[rx] && !IR_state[tx])
      //if we are moving up or we have reached the top, change direction to down but only if IR is not blocked
      if(((direction == up) || (steps_counted <= 0)) && (!IR1_state[tx1] && !IR1_state[tx2] && !IR1_state[rx1] && !IR1_state[rx2])){
        direction = down;                 //change dir to down
        digitalWrite(S1_dir, direction); //0 up, 1 down
      }
      //move up if blocked and down only if not blocked
      if((direction == down) || ((direction == up) && !(steps_counted <= 0))){
        if(direction == up){ //when moving up, move up slower
          steps_counted--;
          move(1,5000); //very slow up movement
        }       
        else{                //down movement at normal speed
          steps_counted++;
          move(1,pulsetime);
        }
      }
      
      //if we made the number of calibrated steps for down movement, we are done
      if(steps_counted >= steps_to_close + 5) done = 1;}} //add a few more steps to counter drift
  
  //done with movement
  return steps_counted;
}

//I2C receive instructions
void receiveEvent(int bytes_incoming){
  uint8_t inputbuffer[7] = {0,0,0,0,0,0,0};
  uint8_t option;
  
  //collect bytes from I2C
  for(uint8_t i = 0; i < bytes_incoming; i++){
    uint8_t c = Wire.read();
    inputbuffer[i] = c;
  }
  
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
  
  if(option == 1){}
  
  if(option == 2){ //move door with simple feedback
    Q_movesimple[1] = inputbuffer[1];                           //direction
    Q_movesimple[2] = inputbuffer[2];                           //target
    Q_movesimple[3] = 0;                                       //clear, because bit-shifting
    Q_movesimple[3] = Q_movesimple[3] | inputbuffer[4];         //pulsetime
    Q_movesimple[3] = (Q_movesimple[3] << 8) | inputbuffer[3];  //pulsetime
    
    Q_movesimple[0] = 1; //queues movement
    newcommand = 1;
  }
  
  if(option == 3){}
  if(option == 4){}
}            

//I2C send instructions
void sendData(){
  uint8_t sendbuffer[2] = {0,0}; //buffer for sending data over I2C
  //potentially send different datapacks
  //  if(selectdata == x)
  //  {sendbuffer1 || sendbuffer2}
  
  //sendbuffer[0] IR status
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[tx1] & 0x01) << 0);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[tx2] & 0x01) << 1);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[rx1] & 0x01) << 2);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[rx2] & 0x01) << 3);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[top] & 0x01) << 4);
  sendbuffer[0] = sendbuffer[0] | ((IR1_state[bottom] & 0x01) << 5);
  //sendbuffer[0] = sendbuffer[0] | ((? & 0x01) << 6);
  //sendbuffer[0] = sendbuffer[0] | ((? & 0x01) << 7);
  
  sendbuffer[1] = sendbuffer[1] | ((S1_busy & 0x01) << 0);
  //sendbuffer[1] = sendbuffer[1] | ((S2_busy & 0x01) << 1);  
  
  Wire.write(sendbuffer,2);
}
