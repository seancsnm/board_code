/*Contains the Arduino Uno/atmega328 code for the board which controls the photogate and personnel gate open/closed sensors on the tower. 
 * This board has support for 6 photogate sensors that are high when open, and low when blocked. The following sensors are defined: 
 *  SENSORS0 Register:
 *   TOP_TRIP_SENSOR - indicates when the trolley is  docked at the top of the tower
 *   GRIP_RELEASE_SENSOR - Tripped when the trolley passes through approximately the midpoint of the tower during a drop - provides feedback to other systems so that the bucket-holding magnet can be released at the appropriate time
 *   MISC_1_SENSOR - unused
 *   MISC_2_SENSOR - unused
 *   TOP_VEL_SENSOR - Top velocity chronograph sensor/first to be tripped
 *   BTM_VEL_SENSOR - Bottom velocity chronograph sensor/second to be tripped
 *  
 *  The personnel  gate open/closed sensor is defined as follows
 *  GATE_SWITCH_SIG - high when gate is closed; low when open. Test shouldn't be allowed to commence with this gate open
 *  
 *  Relevant status bits provide state information of the system, and are listed as follows:
 *  STATUS:
 *   TIMER_ACTIVE - high when the chronograph is active and ready to take a velocity measurement
 *   TEST_READY - high when all sensors are good for a drop test and that the the system state is ready to start a drop test
 *   TIMER_TRIPPED - high when the timer has moved from an active/ready state to a "tripped" state, or a measurement complete state
 *  
 *  Relevant control bits provide commands to the module, and are listed as follows:
 *  CTRL
 *   TIMER_RST - Set to 1 to reset and/or set the timer to its active state. Before setting this bit, ensure that the previous time measurement has been successfully recovered by the master device
 *  
 *  Time data is in microseconds and is stored in the TIMEA (MSB) and TIMEB (LSB) registers
 *  
 *  The recommended master-side sequence for running the module in a test setting is as follows:
 *  0) User ensures that no sensors are blocked, and that trolley is docked and ready to go
 *  1) Master device sets the TIMER_RST bit to activate the timer
 *  2) Master device checks the TEST_READY bit to ensure that all sensors are blocked/unblocked appropriately, and that the timer has been activated
 *  3) Master runs test and waits approximate time it takes for drop test to complete, plus 1 second to give the microcontroller time to save data to memory
 *  4) Master checks TIMER_TRIPPED bit to ensure that timer was tripped succesfully
 *  5) Master retrieves data from registers TIMEA and TIMEB, and re-assembles the time value; checks value to ensure it makes sense
 *  6) Start again at (0) for repeated tests
 *
 */

#include <SimpleModbusSlave.h>

//Pins of interest
enum { //PINB
	P_PB0,
	P_PB1,
	P_PB2,
	P_MOSI,
	P_MISO,
	P_SCK,
	P_XTAL1,
	P_XTAL2
};

enum { //PINC
	P_TOP_TRIP_SENSOR,
	P_GRIP_RELEASE_SENSOR,
	P_MISC_1_SENSOR,
	P_MISC_2_SENSOR,
	P_PC4,
	P_PC5,
	P_RST
};

enum { //PIND
	P_RX,
	P_TX,
	P_TOP_VEL_SENSOR,
	P_BTM_VEL_SENSOR,
	P_DE_RE,
	P_PD5,
	P_GATE_SWITCH_SIG,
	P_PD7
};

enum { //SENSORS0
	R_TOP_TRIP_SENSOR,
	R_GRIP_RELEASE_SENSOR,
	R_MISC_1_SENSOR,
	R_MISC_2_SENSOR,
	R_TOP_VEL_SENSOR,
	R_BTM_VEL_SENSOR,
	R_GATE_SWITCH_SIG
};

//TIMEA - MSBs
//TIMEB - LSBs

enum { //STATUS
	R_TIMER_ACTIVE,
	R_TEST_READY,
	R_TIMER_TRIPPED
};

enum { //CTRL 
R_TIMER_RST
};


//Arduino Pin Numbers (Redundant)
#define STARTPIN     2
#define STOPPIN      3
#define DE_REPIN     4

#define MODBUS_BAUD_RATE 115200
#define MODBUS_ADDR 0x03

enum 
{     
  SENSORS0,     
  TIMEA,
  TIMEB,
  STATUS,
  CTRL,        
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array


//Timer Variables
volatile unsigned long startTime, stopTime, dt;
volatile unsigned long tripFlag = 0;
volatile unsigned int startFlag, stopFlag = 0;

//Timer Interrupt Vectors
const uint8_t startInt = digitalPinToInterrupt(STARTPIN);
const uint8_t stopInt = digitalPinToInterrupt(STOPPIN);

/**Pulls in sensor values and puts them in SENSORS0 register
 * 
 */
void UpdateSENSORS0(void)
{
  holdingRegs[SENSORS0] = (PINC & 0b00001111) | ((PIND & 0b00001100) << 2) | ((PIND & 0b01000000) << 0);
  //holdingRegs[SENSORS0] = 0b11000011;
  //holdingRegs[SENSORS0] = PIND & 0b01000000;
}

/** Checks if ctrl bits have been reset and acts accordingly - requires UpdateSENSORS0() to be run first for accurate results
 *  TIMER_RST - ensures all sensor values are in correct states for starting test and resets and activates timer 
 */
void CheckCTRL(void)
{

  //Check TIMER_RST 
  if (((holdingRegs[SENSORS0] & 0b00110000) == 0b00110000) && (holdingRegs[CTRL] & 0b00000001)) { //Top/Btm Velocity sensors unblocked, Top trip sensor blocked and TIMER_RST flag high

    //Reset and enable timer
    startFlag = 0;
    stopFlag = 0;
    tripFlag = 0;
    holdingRegs[TIMEA] = 0;
    holdingRegs[TIMEB] = 0;
    EIFR |= bit(startInt); //clear start interrupt flag if it has been triggered
    attachInterrupt(startInt, StartISR, FALLING); 
    holdingRegs[CTRL] = (holdingRegs[CTRL] & ~0b00000001); //Clear TIMER_RST CTRL bit
    holdingRegs[STATUS] = (holdingRegs[STATUS] | 0b00000001); //Set TIMER_ACTIVE STATUS bit
    holdingRegs[STATUS] = (holdingRegs[STATUS] & ~0b00000100); //Clear TIMER_TRIPPED STATUS bit
  }
}

/** Updates status registers according to current sensor states - requires UpdateSENSORS() routine first for accurate results
 *  TEST_READY - high when all parameters are set for test to begin; appropriate sensors must be blocked/unblocked, timer must be active but not tripped
 */
void CheckSTATUS(void)
{
  //Update TEST_READY
  /* Registers must be:
   * SENSORS0: GATE_SWITCH_SIG 1, BTM_VEL_SENSOR 1, TOP_VEL_SENSOR 1, GRIP_RELEASE_SENSOR 1, TOP_TRIP_SENSOR 0
   * STATUS: TIMER_ACTIVE 1,  TIMER_TRIPPED 0
   */
  if ( ((holdingRegs[SENSORS0] & 0b01110011) == 0b01110010) && ((holdingRegs[STATUS] & 0b00000101) == 0b00000001) ) {
    holdingRegs[STATUS] = holdingRegs[STATUS] | (0b00000010); //Test is ready
  }
  else {
    holdingRegs[STATUS] = holdingRegs[STATUS] & ~(0b00000010); //Test is not ready - plz don't start test
  }
}


void UpdateTIMER(void)
{
  if (tripFlag) {
    dt = stopTime - startTime;
    holdingRegs[TIMEA] = (uint16_t )(dt >> 16); //places MSBs
    holdingRegs[TIMEB] = (uint16_t)(dt); //places LSBs
  }
}

void StartISR()
{
  startTime = micros();
  detachInterrupt(startInt);
  startFlag = 1;
  EIFR |= bit(stopInt); //clear stop interrupt flag if it has been triggered
  attachInterrupt(stopInt, StopISR, FALLING);
}

void StopISR()
{
  stopTime = micros();
  detachInterrupt(stopInt);
  //dt = stopTime - startTime;
  stopFlag = 1;
  tripFlag = 1;
  holdingRegs[STATUS] = (holdingRegs[STATUS] | 0b00000100); //Set TIMER_TRIPPED STATUS bit
  holdingRegs[STATUS] = (holdingRegs[STATUS] & ~0b00000001); //Clear TIMER_ACTIVE STATUS bit
}

void setup() {
  //Set pinmodes:
  DDRC = 0x00; //All inputs
  DDRD = 0b00000010; //All inputs except TX

  //Setup Timer interrupts
  //EIFR |= bit(startInt); //clear start interrupt flag if it has been triggered (unecessary at first attach)
  //attachInterrupt(startInt, StartISR, FALLING); 
  
  
  //Setup Serial protocol
  modbus_configure(&Serial, MODBUS_BAUD_RATE, SERIAL_8N2, MODBUS_ADDR, DE_REPIN, HOLDING_REGS_SIZE, holdingRegs);
  holdingRegs[STATUS] = 0b00000000; //Clear holdingRegs...
}


void loop() {

  /*Run register update functions in following sequence:
   * UpdateSENSORS0(); - updates the SENSORS0 register, which contains values of all photogate sensors and the personel entry gate
   * CheckCTRL(); - updates the arduino status based on CTRL register status, including resetting timer
   * CheckSTATUS(); - updates the status of the STATUS register bit TEST_READY to determine whether test is ready to run
   * UpdateTIMER(); - if the timer has been tripped, computes the dt and updates the TIMEA and TIMEB registers with that time
   */

  
  UpdateSENSORS0();
  CheckCTRL();
  CheckSTATUS();
  UpdateTIMER();
  modbus_update();
  //delay(1);
}



