/*Contains auxiliary functionality for system, including
 * -control of enable pin
 * -control of electromagnets
 * 
 * Control sequence for this module is as follows: Device boots up with electromagnets turned off. After booting up, the master device may turn electromagnets on and off
 * (even if device not enabled). Once the device is enabled and the TEST_ACTIVE bit has been set, the module is 'armed' to turn off electromagnet 2 with a specified time 
 * delay after the GRIP_RELEASE_SENSOR has been tripped. The time delay can be programmed through the GRIP_RELEASE_TIME register. The global enable signal is also 
 * controlled by this module. The PI_EN bit in the CTRL register must be set by the master device, and the GATE_SWITCH_SIG must be high (entrance gate must be closed) for the 
 * global enable signal to be set. The STATUS register bits TOP_TRIP_SENSOR, GATE_SWITCH_SIG, and GRIP_RELEASE_SENSOR are inputs from arduino. 
 * 
 * An example master control sequence is as follows:
 * -Set GRIP_RELEASE_TIME register to desired time delay in milliseconds
 * -Activate EM1 and EM2
 * -Close gripper
 * -User must ensure gate is closed
 * -Set PI_EN bit in control register
 * -Ensure that EN bit in status register is high
 * -Dock trolley
 * -Set TEST_ACTIVE bit in CTRL register
 * -Deactivate EM1 to drop trolley
 * -Trolley will pass through GRIP_RELEASE_SENSOR and EM2 will deactivate after given time delay from GRIP_RELEASE_SENSOR being tripped
 * -Turn off PI_EN bit to disable testing mode
 * 
 * Notes/considerations
 * -How will user ensure electromagnet is not deactivated when trolley is docked?
 * 
 */

#include <SimpleModbusSlave.h>

/**Functionality, Inputs, Outputs
 * 
 * 
 * 
 * INPUTS
 * -TOP_TRIP_SENSOR
 * -GATE_SWITCH_SIG
 * 
 * OUTPUTS
 * -ENABLE (global)
 * -EM1
 * -EM2
 * 
 */

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
  P_PC1,
  P_PC2,
  P_PC3,
  P_PC4,
  P_GRIP_RELEASE_SENSOR,
  P_RST
};

enum { //PIND
  P_RX,
  P_TX,
  P_EN,
  P_GATE_SWITCH_SIG,
  P_DE_RE,
  P_PD5,
  P_EM1,
  P_EM2
};

enum { //STATUS
  R_EN, //Global enable
  R_TOP_TRIP_SENSOR,
  R_GATE_SWITCH_SIG,
  R_GRIP_RELEASE_SENSOR,
  R_GRIP_TRIPPED //Set TEST_ACTIVE to reset
};

enum { //CTRL
  R_PI_EN,
  R_TEST_ACTIVE, //When on, activates timed grip release 
  R_EM1,
  R_EM2
};


//Digital I/O Pins
#define TOP_TRIP_SENSOR A0
#define GRIP_RELEASE_SENSOR A5
#define ENABLE 2
#define GATE_SWITCH_SIG 3
#define DE_REPIN 4
#define EM1 6
#define EM2 7
#define SS 10 //Slave select
#define MOSI 11
#define MISO 12
#define SCK 13

#define INT_PIN A5 //Interrupt trip pin
#define GR_PCICR_BIT 1 //grip release pin change interrupt control register - set register bit to 1 to enable interrupt
#define GR_PCIFR_BIT 1 //grip release pin change interrupt flag register - when flag set, interrupt is serviced if PCICR bit is set
#define GR_PCMSK_REG 1 //grip release pin change mask register 
#define GR_PCMSK_BIT 5 //grip release pin change mask register bit - determines which pins on interrupt routine are enabled

#define DEFAULT_DELAY_TIME 2000 //grip release sensor delay time, microseconds

#define MODBUS_BAUD_RATE 115200 // Modbus serial baud rate
#define MODBUS_ADDR 0x01

enum 
{     
  STATUS,     
  CTRL,
  GRIP_RELEASE_TIME, //delay time, milliseconds
  TIMER,
  COUNTER,
  DEBUG,
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array



//Timing & util vars
//volatile uint8_t timerActive = 0; //grip sensor to electromagnet release timer is running, should be high
volatile uint8_t tripFlag = 0;
volatile unsigned long gripStartTime, startTime, stopTime, dt; 
uint32_t i = 0;
//volatile unsigned long grip_release_time = DEFAULT_DELAY_TIME; //delay time between grip release sensor getting tripped and gripper electromagnet releasing



ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  PCICR &= ~bit(GR_PCICR_BIT); //disable interrupt
  gripStartTime = millis();
  tripFlag = 1;
  
}  

void UpdateStatus()
{
  //uint8_t _enable = digitalRead(ENABLE);
  uint8_t _top_trip_sensor = digitalRead(TOP_TRIP_SENSOR);
  uint8_t _gate_switch_sig = digitalRead(GATE_SWITCH_SIG);
  uint8_t _grip_release_sig = digitalRead(GRIP_RELEASE_SENSOR);
  uint8_t _test_active = (holdingRegs[CTRL] & bit(R_TEST_ACTIVE)) >> R_TEST_ACTIVE;
  //holdingRegs[DEBUG] = _grip_release_sig;

  if (_test_active) {
    holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(bit(R_GRIP_TRIPPED))) | (0x00 << R_GRIP_TRIPPED); //Reset R_GRIP_TRIPPED flag
  }

  //Updates status register with new values
  holdingRegs[STATUS] = (holdingRegs[STATUS] & 
  ~(0x01 << R_TOP_TRIP_SENSOR | 0x01 << R_GATE_SWITCH_SIG | 0x01 << R_GRIP_RELEASE_SENSOR)) | 
  (_top_trip_sensor << R_TOP_TRIP_SENSOR | _gate_switch_sig << R_GATE_SWITCH_SIG | _grip_release_sig << R_GRIP_RELEASE_SENSOR);

  
}

void UpdateOutputs()
{
  uint8_t _pi_en = (holdingRegs[CTRL] & (0x01 << R_PI_EN)) >> R_PI_EN; 
  uint8_t _em1 = (holdingRegs[CTRL] & (0x01 << R_EM1)) >> R_EM1;
  uint8_t _em2 = (holdingRegs[CTRL] & (0x01 << R_EM2)) >> R_EM2;
  uint8_t _gate_switch_sig = digitalRead(GATE_SWITCH_SIG);
  uint32_t _grip_release_time = holdingRegs[GRIP_RELEASE_TIME];
  uint8_t _en = 0;

  
  digitalWrite(EM2, _em2);
  holdingRegs[DEBUG] = _em2;

  //if (_pi_en && _gate_switch_sig) _en = 1; //Update global enable pin
  if (_pi_en) _en = 1; //Update global enable pin
  else _en = 0;
  
  

  digitalWrite(ENABLE, _en); //update enable output pin
  holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(0x01 << R_EN)) | (_en << R_EN); //Update enable bit in status register


  if (_en) { //only change electromagnet 1 (trolley docking magnet) state when device is enabled
    digitalWrite(EM1, _em1);
  }
  
  if (tripFlag && _en) { //timer tripped - must do blocking trigger of electromagnet
    while((millis() - gripStartTime) < _grip_release_time) {} //Block until trigger time elapsed
    digitalWrite(EM2, 0); //Turn off electromagnet
    //holdingRegs[CTRL] &= ~bit(R_EM1); //turn off electromagnet in control register 
    holdingRegs[CTRL] = (holdingRegs[CTRL] & ~bit(R_EM2)); //Turn off electromagnet in control register
    //reset interrupt

    //de-activate test active flags
    tripFlag = 0;
    holdingRegs[CTRL] &= ~bit(R_TEST_ACTIVE);
    holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(bit(R_GRIP_TRIPPED))) | (0x01 << R_GRIP_TRIPPED); //Set R_GRIP_TRIPPED flag
  }
  else if (holdingRegs[CTRL] & bit(R_TEST_ACTIVE) && _en) { //Test is active - make sure interrupt  is enabled
    PCMSK1 |= bit(GR_PCMSK_BIT); //Set register mask
    PCIFR |= bit(GR_PCIFR_BIT); //clear outstanding  interrupt flags
    PCICR |= bit(GR_PCICR_BIT); //Enable interrupt
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(TOP_TRIP_SENSOR, INPUT);
  pinMode(GRIP_RELEASE_SENSOR, INPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(GATE_SWITCH_SIG, INPUT);
  pinMode(EM1, OUTPUT);
  pinMode(EM2, OUTPUT);
  
  for (i = 0; i < HOLDING_REGS_SIZE; i++) { //Clear holdingRegs...
    holdingRegs[i] = 0x0000; 
  }
  
  modbus_configure(&Serial, MODBUS_BAUD_RATE, SERIAL_8N2, MODBUS_ADDR, DE_REPIN, HOLDING_REGS_SIZE, holdingRegs);
  holdingRegs[GRIP_RELEASE_TIME] = DEFAULT_DELAY_TIME; 
  //Serial.begin(115200);
  //Serial.println("Hello ");
  
}



void loop() {
  // put your main code here, to run repeatedly:
  startTime = micros();
  UpdateStatus();
  UpdateOutputs();
  modbus_update();
  stopTime = micros();
  dt = stopTime - startTime;
  //dt = 1024;
  holdingRegs[TIMER] = dt;
  holdingRegs[COUNTER] = i;

  i++;
  

}
