/* Runs the motors and solenoid valves on the device. 
 * This board has support for 4 solenoids (2 independent, 3 & 4 controlled by single pin). Solenoid voltage is controlled by a jumper, which can provide either 12 or 24 volts. 
 * The solenoids should open when high and closed when low. 
 * 
 * The module supports 2 motors. It uses the DRV8704 motor controller, which has an adjustable current limit feature. The current and motor drive direction can remotely be 
 * controlled. Motor 1 - the winch motor - is setup so that it will only raise while the TOP_TRIP_SENSOR pin is low. Both motors will only operate when the EN pin is active. 
 * 
 * Register and register bit functions are described in their enumerations/declarations
 * 
 * Example control sequence is as follows:
 *  0) User ensures device is enabled by making sure gate is closed and master device has PI_EN bit set on auxiliary module
 *  1) Master device sets current limit on device for trolley motor
 *  2) User may switch to manual control mode to raise/lower the winch using the remote control (current code setup may have safety hazard, as this requires that enable signal
 *  be high to control the motors manually or automatically. Fix may be to allow the user to manually control the winch when device is not enabled until the top trip sensor is
 *  tripped, since that will be when trolley docks to the magnet, and when lowering the winch places the trolley's load on the magnet. Will have to think about this more... Maybe 
 *  allow user to continue raising winch once docking sensor is tripped, but not lower it, in order to ensure that it is correctly seated on magnet? 
 *  3) Once trolley is docked, ensure all personnel are clear from area
 *  4) Lower winch to release trolley from winch, then raise to clear hook from trolley
 *  5) Master device may initiate test, which means activating solenoids immediately after releasing electromagnet. The timing may need to occur on the microcontroller side
 *  instead of the master side... Something like 15 ms delay will exist. 
 *  6) De-activate solenoid valves after specified period of time 
 *  
 * 
 *  
 */

#include "SPI.h"
#include <SimpleModbusSlave.h>

//Atmega328 Register Adresses

enum { //PINB
  P_CTRL_RST,
  P_MTR_CTRL_MODE,
  P_PB2,
  P_MOSI,
  P_MISO,
  P_SCK,
  P_XTAL1,
  P_XTAL2
};

enum { //PINC
  P_POT_A,
  P_POT_B,
  P_CTRL_AIN1,
  P_CTRL_AIN2,
  P_CTRL_BIN1,
  P_CTRL_BIN2,
  P_RST
};

enum { //PIND
  P_RX,
  P_TX,
  P_EN,
  P_TOP_TRIP_SENSOR,
  P_DE_RE,
  P_SOL_A,
  P_SOL_B,
  P_SOL_CD
};



//Registers
enum { //STATUS
  R_EN, //Global enable signal, active high
  R_TOP_TRIP_SENSOR, //Dock sensor, low when docked, high when undocked
  R_MTR_CTRL_SW, //Switches between remote control mode (low) and master control mode (high)
  R_M_A1, //Motor A-1 pin state
  R_M_A2, //Motor A-2 pin state
  R_M_B1, //Motor B-1 pin state
  R_M_B2, //Motor B-2 pin state
  SPI_ERR0, //SPI error code 1 (failed to set ISENSE register)
  SPI_ERR1, //SPI error code 2 (failed to set TORQUE)
  SPI_ERR2, //SPI error code 3 (inactive)
  SPI_ERR3 //SPI error code 4 (inactive)
};

//POTS 
//POTA: 0-7 (potentiometer A ADC reading, truncated to 8 bits)
//POTB: 8-15 (potentiometer B ADC reading, truncated to 8 bits)

//CUR_CTRL
//M_CUR: 0-7

enum { //DEV_CTRL
  R_M_A_VEL, //Motor A 'velocity' (only works with 0 (don't move) or 1 (fully on)
  R_M_A_DIR, //Motor A direction: 1 = forward; 0 = reverse
  R_M_B_VEL, //Motor B 'velocity' (only works with 0 (don't move) or 1 (fully on)
  R_M_B_DIR, //Motor B direction: 1 = forward; 0 = reverse
  R_DEV_CTRL_4, //Unused/spare
  R_SOL_A, //keep at 5; Solenoid A control bit (0 = off, 1 = on)
  R_SOL_B, //keep at 6; Solenoid B control bit (0 = off, 1 = on)
  R_SOL_CD, //keep at 7; Solenoid C & D control bit (0 = off, 1 = on)
  R_M1_SAFE // Determines whether the docking sensor prevents M1 from moving in upward direction if tripped. If high, safety enabled
};

enum //Register Addressing
{     
  STATUS,
  POTS,
  CUR_CTRL,
  TORQUE,
  DEV_CTRL,
  TIMER,
  COUNTER,
  DEBUG,      
  HOLDING_REGS_SIZE // leave this one
  // total number of registers for function 3 and 16 share the same register array
  // i.e. the same address space
};

//Digital I/O Pins
#define POT_A A0
#define POT_B A1
#define CTRL_AIN1 A2
#define CTRL_AIN2 A3
#define CTRL_BIN1 A4
#define CTRL_BIN2 A5
#define ENABLE 2
#define TOP_TRIP_SENSOR 3
#define DE_REPIN 4
#define SOL_A 5
#define SOL_B 6
#define SOL_CD 7
#define MTR_CTRL_MODE 9
#define SS 10 //Slave select
#define MOSI 11
#define MISO 12
#define SCK 13

#define MODBUS_BAUD_RATE 115200
#define MODBUS_ADDR 0x02

//Motor Control Address Limits
#define ADDR_LOW B000
#define ADDR_HIGH B111

//Initial Motor Current Limits - note can only set one current limit per motor controller
#define M_CUR_INIT 10 //desired current limit in 10ths of an amp (80=8.0 amps)
#define ISGAIN 10 //Hard-coded to be set to 10 V/V
#define RISENSE 0.0075 //Current sense resistor value [ohms] (originally 0.0150)
#define FWD 1 //Forward motor direction
#define REV 0 //Reverse motor direction
#define POT_FWD_TH 200 // Threshold for motor driving forward
#define POT_REV_TH 64 // Threshold for motor in reverse direction

#define SPI_WRITE_DELAY 2 //Write delay between spi writes, ms
#define SPI_SPEED 2500000 //SPI speed in Hz


#define N_CHECK 3 //number of times to read a register

unsigned int holdingRegs[HOLDING_REGS_SIZE]; // function 3 and 16 register array
SPISettings DRV8704Settings (SPI_SPEED, MSBFIRST, SPI_MODE3); //Create SPI object

//Function Prototypes 1
uint16_t ReadReg(uint8_t addr);
uint16_t WriteReg(uint8_t addr, uint16_t data, uint16_t mask);
int RedundantReadReg(uint8_t addr, uint16_t mask);
int RedundantWriteReg(uint8_t addr, uint16_t data, uint16_t mask);


//Motor Class
/*Motor class - contains motor "object" for a single motor controlled through the DRV8704 motor driver
 * Motor(M1, M2, top_lim, btm_lim) is constructor that accepts motor A and B pins, and top and bottom limit switch registers. Point to null if not available
 */
class Motor //: public Stream
{
  private:
    
    //static uint8_t top_lim, btm_lim; //limit switches, active low
  public:
    volatile uint8_t M1, M2; //Motor input pins 
    volatile uint8_t vel, dir; //velocity (0-255) and direction (0=reverse, 1=forward) to command the motor
    uint8_t current_lim; //Current limit, in deci-amps (i.e. 105 = 10.5 A)  

    Motor(uint8_t _M1, uint8_t _M2);
    void Drive(uint8_t _vel, uint8_t _dir, uint8_t _fwd_lim, uint8_t _rev_lim);
    void SetCurrent(uint8_t _current_lim);
};

Motor::Motor(uint8_t _M1, uint8_t _M2)
{
  M1 = _M1;
  M2 = _M2;
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  //SetCurrent(M_CUR_INIT); //Set controller to hard-coded current limit
  //top_lim = _top_lim;
  //btm_lim = _btm_lim;
}

void Motor::Drive(uint8_t _vel, uint8_t _dir, uint8_t _fwd_lim, uint8_t _rev_lim)
{
  vel = _vel;
  dir = _dir; 
  //case 1: motor off or has hit limit switch
  if (vel == 0 || (vel > 0 && dir == FWD && !_fwd_lim) || (vel > 0 && dir == REV && !_rev_lim)) {
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
  }
  //case 2: motor forward
  else if (dir == FWD) {
    //analogWrite(M1, vel);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
  }
  //case 3: motor reverse
  else if (dir == REV) {
    //analogWrite(M2, vel);
    digitalWrite(M2, HIGH);
    digitalWrite(M1, LOW);
  }  
}

void Motor::SetCurrent(uint8_t _current_lim) 
{
  current_lim = _current_lim;
  uint8_t Torque = (uint8_t)((((uint32_t)current_lim * 256 * (uint32_t)ISGAIN) * (float)RISENSE) / (27.5) + 0.5); //Apply torque equation, round by adding 0.5
  holdingRegs[TORQUE] = Torque; //Assign computed torque
  SPI.beginTransaction(DRV8704Settings);
  if (RedundantWriteReg(0, 0B000100000001, 0B111111111111)) { //Set ISENSE gain to B01 (10V/V) 000100000001
    holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(0x01 << SPI_ERR0)) | (0x01 << SPI_ERR0); //Report error as SPI_ERR0
  }
  delay(SPI_WRITE_DELAY);
  if (RedundantWriteReg(1, Torque, 0xFF)) { //Set TORQUE register to computed torque for desired current
    holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(0x01 << SPI_ERR1)) | (0x01 << SPI_ERR1); //Report error as SPI_ERR1
  }
  //Set current procedure
  SPI.endTransaction(); 
  holdingRegs[CUR_CTRL] = _current_lim;
}

//Function Prototypes 2
void UpdateStatus(Motor MA, Motor MB);
void UpdateOutputs(Motor MA, Motor MB);




/** Updates status registers
 *  
 */
void UpdateStatus(Motor MA, Motor MB)
{
  // Update motor status pins
  //holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(bit(R_M_A1) | bit(R_M_A2) | bit(R_M_B1) | bit(R_M_B2))) | ( MA.M1 << R_M_A1 | MA.M2 << R_M_A2 | MB.M1 << R_M_B1 | MB.M2 << R_M_B2);
  //holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(0x01 << R_MTR_CTRL_SW) | digitalRead(MTR_CTRL_MODE)); //motor control mode (!!!updated in UpdateOutputs()!!!)
  //holdingRegs[STATUS] = (MA.M1 << R_M_A1 | MA.M2 << R_M_A2 | MB.M1 << R_M_B1 | MB.M2 << R_M_B2);
  //holdingRegs[STATUS] = MB.M2;
  //holdingRegs[STATUS] = LOW;
  
  //get motor pin info
  //uint8_t mtr_status = (PINC & (bit(P_CTRL_AIN1) | bit(P_CTRL_AIN2) | bit(P_CTRL_BIN1) | bit(P_CTRL_BIN2))) << 1; //Pull from PINC register, leftshift to fit STATUS reg
  uint8_t mtr_status = (digitalRead(CTRL_AIN1) << R_M_A1 | digitalRead(CTRL_AIN2) << R_M_A2 | digitalRead(CTRL_BIN1) << R_M_B1 | digitalRead(CTRL_BIN2) << R_M_B2);
  holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(bit(R_M_A1) | bit(R_M_A2) | bit(R_M_B1) | bit(R_M_B2))) | mtr_status;
  
  
  //holdingRegs[STATUS] = mtr_status;

  //get top trip sensor status
  holdingRegs[STATUS] = ((holdingRegs[STATUS] & ~(0x01 << R_TOP_TRIP_SENSOR)) | digitalRead(TOP_TRIP_SENSOR) << R_TOP_TRIP_SENSOR); // top trip sensor
}

/** Updates the motor and solenoid outputs based on register values each loop
 *  
 */
void UpdateOutputs(Motor MA, Motor MB)
{
  uint8_t drive_velA, drive_dirA, drive_velB, drive_dirB;
  uint8_t _potA = (analogRead(POT_A) >> 2); //Truncate pot reading to 8-bit
  uint8_t _potB = (analogRead(POT_B) >> 2); //Truncate pot reading to 8-bit
  uint8_t _mtr_ctrl_sw = digitalRead(MTR_CTRL_MODE);
  uint8_t _enable = digitalRead(ENABLE);
  uint8_t _top_trip_sensor = digitalRead(TOP_TRIP_SENSOR);
  uint8_t _m1_safe = (holdingRegs[DEV_CTRL] & bit(R_M1_SAFE)) >> R_M1_SAFE; 
  

  holdingRegs[POTS] = ((_potA ) | ((uint16_t)(_potB ) << 8)); //concetenate pot inputs, store values in register (should truncate to 8 bits)
  //holdingRegs[POTS] = ((_potA);
  holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(0x01 << R_MTR_CTRL_SW) | (_mtr_ctrl_sw << R_MTR_CTRL_SW)); //store mtr ctrl sw value in register
  holdingRegs[STATUS] = (holdingRegs[STATUS] & ~(0x01 << R_EN)) | (_enable << R_EN); //Store enable status 
  if ((_mtr_ctrl_sw) && _enable) { //control input switch high and enabled - master control mode
    drive_velA = (holdingRegs[DEV_CTRL] & (0x01 << R_M_A_VEL)) >> R_M_A_VEL; 
    drive_dirA = (holdingRegs[DEV_CTRL] & (0x01 << R_M_A_DIR)) >> R_M_A_DIR; 
    drive_velB = (holdingRegs[DEV_CTRL] & (0x01 << R_M_B_VEL)) >> R_M_B_VEL;
    drive_dirB = (holdingRegs[DEV_CTRL] & (0x01 << R_M_B_DIR)) >> R_M_B_DIR;  
  }
  else if (!_mtr_ctrl_sw) { // potentiometer control mode
    //Set motor A direction
    if (_potA > POT_FWD_TH) {
      drive_velA = 1;
      drive_dirA = 1;
    }
    else if (_potA < POT_REV_TH) {
      drive_velA = 1;
      drive_dirA = 0;
    }
    else {
      drive_velA = 0;
      drive_dirA = 0;
    }

    //Set motor B direction
    if (_potB > POT_FWD_TH) {
      drive_velB = 1;
      drive_dirB = 1;
    }
    else if (_potB < POT_REV_TH) {
      drive_velB = 1;
      drive_dirB = 0;
    }
    else {
      drive_velB = 0;
      drive_dirB = 0;
    }   
  }

  else {
    drive_velA = 0;
    drive_velB = 0;
  }

  //Drive motors 
  MA.Drive(drive_velA, drive_dirA, (_top_trip_sensor || !_m1_safe), 1);
  MB.Drive(drive_velB, drive_dirB, 1, 1);
  
  //Set solenoid states
  
  if (_enable) {
    digitalWrite(SOL_A, (holdingRegs[DEV_CTRL] & bit(R_SOL_A)) >> R_SOL_A);
    digitalWrite(SOL_B, (holdingRegs[DEV_CTRL] & bit(R_SOL_B)) >> R_SOL_B);
    //digitalWrite(6, HIGH); //debugging
    digitalWrite(SOL_CD, (holdingRegs[DEV_CTRL] & bit(R_SOL_CD)) >> R_SOL_CD);
  }
  else {
    digitalWrite(SOL_A, 0);
    digitalWrite(SOL_B, 0);
    digitalWrite(SOL_CD, 0);
  }
  
  /*if (_enable) { //For some reason, causes output pin to oscillate with square wave
    uint16_t sol_vals = (PIND & ~(bit(P_SOL_A) | bit(P_SOL_B) | bit(P_SOL_CD))) | (holdingRegs[DEV_CTRL] & (bit(R_SOL_A) | bit(R_SOL_B) | (R_SOL_CD))); //set solenoid bits from register
    //PIND = (PIND & ~(bit(P_SOL_A) | bit(P_SOL_B) | bit(P_SOL_CD))) | sol_vals;
    //PIND = (PIND & ~(bit(P_SOL_A) | bit(P_SOL_B) | bit(P_SOL_CD))) | (holdingRegs[DEV_CTRL] & (bit(R_SOL_A) | bit(R_SOL_B) | (R_SOL_CD))); //set solenoid bits from register
    PIND = sol_vals;
  }
  else {
    PIND = (PIND & ~(bit(P_SOL_A) | bit(P_SOL_B) | bit(P_SOL_CD))) | (0x0000 & (bit(R_SOL_A) | bit(R_SOL_B) | (R_SOL_CD))); //clear all solenoid bits
  }*/

  /*
  //if ((PIND & 1 << P_EN) == 1 << P_EN) { //If device enabled
  if (_enable) { //If device enabled
    //uint8_t sol_vals = holdingRegs[DEV_CTRL] & (bit(R_SOL_A) | bit(R_SOL_B) | (R_SOL_CD))
    PIND = (PIND & ~(bit(P_SOL_A) | bit(P_SOL_B) | bit(P_SOL_CD))) | (holdingRegs[DEV_CTRL] & (bit(R_SOL_A) | bit(R_SOL_B) | (R_SOL_CD))); //set solenoid bits
    
    //digitalWrite(SOL_A, holdingRegs[DEV_CTRL] & (0x01 << R_SOL_A)); //Solenoid A on/off
    //digitalWrite(SOL_B, holdingRegs[DEV_CTRL] & (0x01 << R_SOL_B)); //Solenoid B on/off
    
    MA.Drive(R_M_A_VEL, R_M_A_DIR, TOP_TRIP_SENSOR, 1); 
    MB.Drive(R_M_B_VEL, R_M_B_DIR, 1, 1);  
  }
  else { //device not enabled
    //digitalWrite(SOL_A, 0);
    //digitalWrite(SOL_B, 0);

    MA.Drive(0, R_M_A_DIR, TOP_TRIP_SENSOR, 1); 
    MB.Drive(0, R_M_B_DIR, 1, 1); 
  }*/

  //Update status register with motor output states (done in UpdateStatus() routine, not here!!!!)
  //holdingRegs[STATUS] &= (~(bit(R_M_A1) | bit (R_M_A2) | bit(R_M_B1) | bit(R_M_B2)) | (MA.M1 << R_M_A1 | MA.M2 << R_M_A2 | MB.M1 << R_M_B1 | MB.M2 << R_M_B2)); 

  if (MA.current_lim != holdingRegs[CUR_CTRL]) { //Check if current limit needs to be updated
    MA.SetCurrent(holdingRegs[CUR_CTRL]); //only need to do this once since MA and MB use same controller
  }   
}


/* Reads a 12-bit register value at the given address 
 *  @param addr the 3-bit address of the register to read from
 *  @returns 12-bit value from register
 */
uint16_t ReadReg(uint8_t addr)
{
  delay(1);
  if ((addr > ADDR_HIGH) || (addr < ADDR_LOW)) {
    return 0; //Invalid address - cannot return meaningful data
  }
  uint8_t msb, lsb, input;
  input = (1 << 7) | (addr << 4); // Read byte is in form 0b(Raaa xxxx)where aaa=address
  digitalWrite(SS, HIGH);
  msb = SPI.transfer(input);
  //delay(10);
  lsb = SPI.transfer(input);
  digitalWrite(SS, LOW);
  return (((0x0F & msb) << 8) | lsb);
}

void PrintReg(uint8_t addr)
{
  int result = ReadReg(addr);
  Serial.print("@(");
  Serial.print(addr, HEX);
  Serial.print(")\t");
  Serial.println(result, BIN);
}

/* Sets a 12-bit register at addresss addr using data. mask specifies which bits to replace.
 *  @param addr the 3-bit address to write data to
 *  @param data the 12-bit data to write at address
 *  @param mask specifies which bits to replace (e.g. B00001111 specifies that last 4 bits are replaced with data) 
 */
uint16_t WriteReg(uint8_t addr, uint16_t data, uint16_t mask)
{
   uint8_t msb, lsb;
   //read in old data
   uint16_t oldData = ReadReg(addr);
   //Compute new data
   //Serial.println(oldData);
   //Serial.println(~mask);
   //Serial.println(data);
   uint16_t newData = (oldData & ~mask) | data; //Apply mask to get value that is pushed
   newData = (0 << 15 | addr << 12 | newData);
   //Serial.println(newData);
   msb = (newData & 0xFF00) >> 8; //Explicitly showing elimination of rightmost bits
   lsb = (newData & 0x00FF); //Explicitly showing elimination of leftmost bits

   //Serial.println(msb);
   //Serial.println(lsb);
   //Write data
   delay(SPI_WRITE_DELAY); 
   digitalWrite(SS, HIGH);
   SPI.transfer(msb);
   SPI.transfer(lsb); 
   digitalWrite(SS, LOW);
}

/* Reads a register 3 times and repeats up to N times if all 3 reads are not identical. Retuns -1 if all sets of reads fail 
 *  
 *  @param mask is used for determining which bits matter when comparing successive reads. Useful when it is possible bits will change between reads
 */
int RedundantReadReg(uint8_t addr, uint16_t mask)
{
  uint8_t i, j;
  //uint8_t success_flag = 0; 
  int regVal; 
  for (i = 0; i < N_CHECK; i++) {
    //int read_count = 1;
    regVal = ReadReg(addr); //First register read
    uint8_t success_flag = 1;
    for (j = 0; j < 2; j++) {
      if ((ReadReg(addr) & mask) != (regVal & mask)) {
        success_flag = 0; //read set failed
        break; //give up 
      }
    }
    if (success_flag == 1) return regVal; //Successful read set, so finished
  }
  return -1; //all read sets unsuccessful, so failed
}

/* Writes a register then reads it 3 times and repeats up to N times if all 3 reads are not identical. Retuns -1 if all sets of reads fail 
 *  Tries to ensure that a correct register value is written
 */
int RedundantWriteReg(uint8_t addr, uint16_t data, uint16_t mask)
{
  int i;
  //read in old data
  uint16_t oldData = RedundantReadReg(addr, mask); 
  if (oldData == -1) return -1; //initial read attempt failed

  //Compute new data
  uint16_t newData = (oldData & ~mask) | data; //Apply mask to get value that is pushed
  newData = (0 << 15 | addr << 12 | newData); //Add in address(?)
  //Serial.println(newData);
  //Split packet
  uint16_t msb = (newData & 0xFF00) >> 8; //Explicitly showing elimination of rightmost bits
  uint16_t lsb = (newData & 0x00FF); //Explicitly showing elimination of leftmost bits

  //Attempt to write data until max number of attempts has been reached or success
  for (i = 0; i < N_CHECK; i++) {
    //Write attempt
    delay(SPI_WRITE_DELAY); 
    digitalWrite(SS, HIGH);
    SPI.transfer(msb);
    SPI.transfer(lsb); 
    digitalWrite(SS, LOW);

    //Readback atttempt
    if ((RedundantReadReg(addr, mask) & mask) == (newData & mask)) return 0; //done if read successful  
    holdingRegs[DEBUG]++;
  }

  return -1; //If all attempts fail, error code
}

uint8_t SPI_addr;
uint8_t stat, val1, val2;
uint16_t result;
//uint32_t i = 0;

Motor MA = Motor(CTRL_AIN1, CTRL_AIN2);
Motor MB = Motor(CTRL_BIN1, CTRL_BIN2);

void setup() {
  int i;
  for (i = 0; i < HOLDING_REGS_SIZE; i++) { //Clear holdingRegs...
    holdingRegs[i] = 0x0000; 
  }
  //holdingRegs[TIMER] = 10000;
  
  // put your setup code here, to run once:
  pinMode(SOL_A, OUTPUT);
  pinMode(SOL_B, OUTPUT);
  pinMode(SOL_CD, OUTPUT);
  pinMode(ENABLE, INPUT);
  pinMode(CTRL_AIN1, OUTPUT);
  pinMode(CTRL_AIN2, OUTPUT);
  pinMode(CTRL_BIN1, OUTPUT);
  pinMode(CTRL_BIN2, OUTPUT);
  
  SPI.begin();

  MA.SetCurrent(M_CUR_INIT); //Set controller to hard-coded current limit
  modbus_configure(&Serial, MODBUS_BAUD_RATE, SERIAL_8N2, MODBUS_ADDR, DE_REPIN, HOLDING_REGS_SIZE, holdingRegs);
  
  /**Setup sequence
  -Create motor objects
  -initialize motor with current setting
  -initialize modbus with DIO 4 as DE/RE pin
  -set initial values for status registers?
  -
  */
}


uint32_t startTime, stopTime, dt;

uint32_t i = 0;
void loop() {
  // put your main code here, to run repeatedly:
  //Device Update Loop
  startTime = millis();
  UpdateOutputs(MA, MB); //Updates motors, solenoids, and related registers
  
  UpdateStatus(MA, MB); //Updates the status register
  
  modbus_update();
  
  
  //if (dt > holdingRegs[TIMER]) holdingRegs[TIMER] = dt;
  
  holdingRegs[COUNTER] = i;
  stopTime = millis();
  (dt = stopTime - startTime);
  holdingRegs[TIMER] = dt;

  i++;
}
