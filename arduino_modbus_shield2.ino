#include <PID_v1.h>

#include <EEPROM.h>
#include <SimpleModbusSlave.h>

#define DEFAULT_SLAVE_ID 1
#define DEFAULT_BAUDRATE 1

#define BAUDRATES_SIZE 4

unsigned long BAUDRATES[BAUDRATES_SIZE] = {
  2400,
  9600,
  19200,
  57600
};

// RX/TX enable pin
#define TX_PIN 6

//We have 3 1-10 ouptuts on the board
#define OUTPUTS_COUNT 3

//and 1 input
#define INPUTS_COUNT 1

//Analog ouptuts has this value by default
#define DEFAULT_ANALOG_OUT_VAL 0
#define DEFAULT_ANALOG_IN_VAL 0

int CURRENT_SLAVE_ID, CURRENT_BAUDRATE;

// Opapms connected to this pins
uint8_t analogOutPins[OUTPUTS_COUNT] = { 2, 3, 4 };
uint8_t analogInPins[INPUTS_COUNT] = { A15 };

//Holding registers
enum
{
  //1-10 out
  ANALOG_OUT_REG_0,
  ANALOG_OUT_REG_1,
  ANALOG_OUT_REG_2,

  //PID
  PID_CURRENT_SETPOINT_REG,
  PID_DEFAULT_SETPOINT_REG, //stored in EEPROM
  PID_INPUT_REG, //readonly
  PID_KP_REG,
  PID_KI_REG,
  PID_KD_REG,

  //settings
  BAUDRATE_REG,
  SLAVE_ID_REG,

  HOLDING_REGS_SIZE
};

// EEPROM adresses
enum {
  SLAVE_ID_EEPROM_ADDR,
  BAUDRATE_EEPROM_ADDR,
  
  PID_DEFAULT_SETPOINT_EEPROM_HIGH_ADDR,
  PID_DEFAULT_SETPOINT_EEPROM_LOW_ADDR,
 
  PID_KP_EEPROM_HIGH_ADDR,
  PID_KP_EEPROM_LOW_ADDR,
   
  PID_KI_EEPROM_HIGH_ADDR,
  PID_KI_EEPROM_LOW_ADDR,
  
  PID_KD_EEPROM_HIGH_ADDR,
  PID_KD_EEPROM_LOW_ADDR
};

#define PID_INPUT_PIN A15
#define PID_OUTPUT_PIN 5

double pidInput, pidOuptut;

double PID_DEFAULT_SETPOINT;
double PID_KP, PID_KI, PID_KD;

int foo = fetchModbusSettings();
int bar = fetchPidSettings();

int PID_DISABLED_SETPOINT_VALUE = -32767;
double PID_CURRENT_SETPOINT = PID_DEFAULT_SETPOINT;

PID pid(&pidInput, &pidOuptut, &PID_CURRENT_SETPOINT, PID_KP, PID_KI, PID_KD, DIRECT);

// data array for modbus network sharing
unsigned int holdingRegs[HOLDING_REGS_SIZE] = {
  //1-10 out
  DEFAULT_ANALOG_OUT_VAL,
  DEFAULT_ANALOG_OUT_VAL,
  DEFAULT_ANALOG_OUT_VAL,

  //PID
  PID_CURRENT_SETPOINT,
  PID_DEFAULT_SETPOINT,
  pidInput,
  PID_KP,
  PID_KI,
  PID_KD,
  
  CURRENT_BAUDRATE,
  CURRENT_SLAVE_ID
};

void setup() {
  Serial.begin(9600);
  startModbus();
  togglePid();
}

void startModbus(){
  Serial.println("Modbus Slave Device Start");

  Serial.print("Baudrate: ");
  Serial.print(BAUDRATES[CURRENT_BAUDRATE]);
  Serial.print(" Slave ID: ");
  Serial.print(CURRENT_SLAVE_ID);
  Serial.print(" Registers count: ");
  Serial.println(HOLDING_REGS_SIZE);
  
  modbus_configure(&Serial3, BAUDRATES[CURRENT_BAUDRATE], SERIAL_8N2, CURRENT_SLAVE_ID, TX_PIN, HOLDING_REGS_SIZE, holdingRegs);
}

void togglePid(){
  //turn the PID on
  if ((int)PID_CURRENT_SETPOINT != (int)PID_DISABLED_SETPOINT_VALUE) {
    pid.SetMode(AUTOMATIC);
    Serial.print("PID Started. ");
    Serial.print("Default setpoint: ");
    Serial.print(PID_DEFAULT_SETPOINT);
    Serial.print(" Kp: ");
    Serial.print(PID_KP);
    Serial.print(" Ki: ");
    Serial.print(PID_KI);
    Serial.print(" Kd: ");
    Serial.println(PID_KD);
  } else {
    pid.SetMode(MANUAL);
    Serial.print("PID Stopped. Because setpoint is: ");
    Serial.println(PID_CURRENT_SETPOINT);
  }
}

void loop() {

  modbus_update();

  // 1-10 inputs
  for (int i = 0; i < OUTPUTS_COUNT; i++)
  {
    int value = map(holdingRegs[i], 0, 255, 0, 240); //restrict range
    analogWrite(analogOutPins[i], value);
  }

  updatePid();
  watchPidSettings();
  watchModbusSettings();
}

void updatePid() {
  if (holdingRegs[PID_CURRENT_SETPOINT_REG] != (int) PID_CURRENT_SETPOINT) {
     PID_CURRENT_SETPOINT = holdingRegs[PID_CURRENT_SETPOINT_REG];
     togglePid();
  }
 
  pidInput = analogRead(PID_INPUT_PIN);
  holdingRegs[PID_INPUT_REG] = pidInput;
  pid.Compute();
  analogWrite(PID_OUTPUT_PIN, pidOuptut);
}

void watchModbusSettings() {
  if (holdingRegs[BAUDRATE_REG] != CURRENT_BAUDRATE || holdingRegs[SLAVE_ID_REG] != CURRENT_SLAVE_ID) {
       //update baudrate and slaveAddr
    saveModbusSettings(holdingRegs[BAUDRATE_REG], holdingRegs[SLAVE_ID_REG]);

    holdingRegs[SLAVE_ID_REG] = CURRENT_SLAVE_ID;
    holdingRegs[BAUDRATE_REG] = CURRENT_BAUDRATE;

    modbus_update_comms(BAUDRATES[CURRENT_BAUDRATE], SERIAL_8N2, CURRENT_SLAVE_ID);
  }
}

void watchPidSettings(){
  if (holdingRegs[PID_KP_REG] !=  PID_KP || holdingRegs[PID_KI_REG] != PID_KI  ||  holdingRegs[PID_KD_REG] != PID_KD ||
    holdingRegs[PID_DEFAULT_SETPOINT_REG] != PID_DEFAULT_SETPOINT)
   {
    
      savePidSettings(holdingRegs[PID_DEFAULT_SETPOINT_REG], holdingRegs[PID_KP_REG], holdingRegs[PID_KI_REG],  holdingRegs[PID_KD_REG]);
      pid.SetTunings(PID_KP, PID_KI, PID_KD);
   }
}

int getFlagAdress(int address) {
  return EEPROM.length() - 1 - address;
}

//We store special flags in the end of eeprom. 
//If flag is settedm that means something where writen.
bool isInEeprom(int address) {
  int flag = EEPROM.read(getFlagAdress(address));

  return flag == 1;
}

int getFromPersistent(int address, int substitute) {
  if (!isInEeprom(address)) {
    return substitute;
  }
  return EEPROM.read(address);
}

void saveToPersistent(int address, int value) {
  EEPROM.update(address, value);
  EEPROM.update(getFlagAdress(address), 1);
}

void saveIntToEEPROM(int value, int startAdress){
  saveToPersistent(startAdress, highByte(value));
  saveToPersistent(startAdress+1, lowByte(value));
}

int readIntFromEEPROM(int startAddress, int substitute){
  if (!isInEeprom(startAddress)) {
    return substitute;
  }
  
  byte highByte = EEPROM.read(startAddress);
  byte lowByte = EEPROM.read(startAddress+1);
  
  return  word(highByte, lowByte);
}

void savePidSettings(double defaultSetpoint, double kp, double ki, double kd) {
  saveIntToEEPROM(defaultSetpoint, PID_DEFAULT_SETPOINT_EEPROM_HIGH_ADDR);
  saveIntToEEPROM(kp*10, PID_KP_EEPROM_HIGH_ADDR);
  saveIntToEEPROM(ki*10, PID_KI_EEPROM_HIGH_ADDR);
  saveIntToEEPROM(kd*10, PID_KD_EEPROM_HIGH_ADDR);
  
  PID_DEFAULT_SETPOINT = defaultSetpoint;
  PID_KP = kp;
  PID_KI = ki;
  PID_KD = kd;
}


int fetchPidSettings(){
  PID_DEFAULT_SETPOINT = readIntFromEEPROM(PID_DEFAULT_SETPOINT_EEPROM_HIGH_ADDR, PID_DISABLED_SETPOINT_VALUE);
  PID_KP = readIntFromEEPROM(PID_KP_EEPROM_HIGH_ADDR, 10) / 10;
  PID_KI = readIntFromEEPROM(PID_KI_EEPROM_HIGH_ADDR, 10) / 10;
  PID_KD = readIntFromEEPROM(PID_KD_EEPROM_HIGH_ADDR, 10) / 10;
  
  return 0;
}

void saveModbusSettings(int baudrate, int id) {
   //save only if baudare inside of available range
  if (baudrate < BAUDRATES_SIZE) {
    CURRENT_BAUDRATE = baudrate;
    saveToPersistent(BAUDRATE_EEPROM_ADDR, baudrate);
  }
  
  saveToPersistent(SLAVE_ID_EEPROM_ADDR, id);
  CURRENT_SLAVE_ID = id;
}

int fetchModbusSettings() {
  CURRENT_BAUDRATE = getFromPersistent(BAUDRATE_EEPROM_ADDR, DEFAULT_BAUDRATE);
  CURRENT_SLAVE_ID = getFromPersistent(SLAVE_ID_EEPROM_ADDR, DEFAULT_SLAVE_ID);
  return 0;
}
