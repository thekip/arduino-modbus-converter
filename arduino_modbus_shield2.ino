#include <EEPROM.h>
#include <SimpleModbusSlave.h>

#define DEFAULT_SLAVE_ID 1
#define DEFAULT_BAUDRATE 1

#define SLAVE_ID_EEPROM_ADDR 0
#define BAUDRATE_EEPROM_ADDR 1

#define BAUDRATES_SIZE 4

unsigned long BAUDRATES[BAUDRATES_SIZE] = {
  2400,
  9600,
  19200,
  57600
};

// RX/TX enable pin
#define TX_PIN 6

//We have 4 OpAmps on the board
#define OPAMPS_COUNT 4

//Analog ouptuts has this value by default
#define DEFAULT_ANALOG_OUT_VAL 0

int CURRENT_SLAVE_ID = getSlaveId();
int CURRENT_BAUDRATE = getBaudrate();

// Opapms connected to this pins
uint8_t opampsPins[OPAMPS_COUNT] = { 2, 3, 4, 5 };
//int analogIn1 = A15;

//Holding regiesters
enum 
{ 
  ANALOG_OUT_REG_0,
  ANALOG_OUT_REG_1,
  ANALOG_OUT_REG_2,
  BAUDRATE_REG,
  SLAVE_ID_REG,
  HOLDING_REGS_SIZE 
};

// data array for modbus network sharing
unsigned int holdingRegs[HOLDING_REGS_SIZE] = {
  DEFAULT_ANALOG_OUT_VAL, 
  DEFAULT_ANALOG_OUT_VAL, 
  DEFAULT_ANALOG_OUT_VAL, 
  CURRENT_BAUDRATE,
  CURRENT_SLAVE_ID
};


void setup() {
  Serial.begin(9600);

  Serial.println("Modbus Slave Device Start");

  Serial.print("Baudrate: ");
  Serial.print(CURRENT_BAUDRATE);
  Serial.print(" Slave ID: ");
  Serial.print(CURRENT_SLAVE_ID);
  Serial.print(" Registers count: ");
  Serial.println(HOLDING_REGS_SIZE);
  
  modbus_configure(&Serial3, BAUDRATES[CURRENT_BAUDRATE], SERIAL_8N2, CURRENT_SLAVE_ID, TX_PIN, HOLDING_REGS_SIZE, holdingRegs);
}

void loop() {
  
  modbus_update();
 
  // set data to opamps
  for (int i = 0; i < OPAMPS_COUNT; i++)
  {
    int value = map(holdingRegs[i], 0, 255, 0, 240); //restrict range
    analogWrite(opampsPins[i], value);
  }

  //update baudrate and slaveAddr
  if (holdingRegs[BAUDRATE_REG] != CURRENT_BAUDRATE || holdingRegs[SLAVE_ID_REG] != CURRENT_SLAVE_ID) {
    CURRENT_SLAVE_ID = holdingRegs[SLAVE_ID_REG];
     
    saveBaudrate(holdingRegs[BAUDRATE_REG]);
    saveSlaveId(holdingRegs[SLAVE_ID_REG]);

    holdingRegs[SLAVE_ID_REG] = CURRENT_SLAVE_ID;
    holdingRegs[BAUDRATE_REG] = CURRENT_BAUDRATE;
    
    modbus_update_comms(BAUDRATES[CURRENT_BAUDRATE], SERIAL_8N2, CURRENT_SLAVE_ID);
  }
}

int getFromPersistent(int address, int substitute) {
  int val = EEPROM.read(address);
  if (val != 255) {
    return val;
  }

  return substitute;
}

void saveBaudrate(int baudrate) {
   //save only if baudare inside of available range
   if (baudrate < BAUDRATES_SIZE) {
    CURRENT_BAUDRATE = baudrate;
    EEPROM.update(BAUDRATE_EEPROM_ADDR, baudrate);
   }
}

void saveSlaveId(int id){
  EEPROM.update(SLAVE_ID_EEPROM_ADDR, id);
  CURRENT_SLAVE_ID = id;
}

int getSlaveId() {
  return getFromPersistent(SLAVE_ID_EEPROM_ADDR, DEFAULT_SLAVE_ID);
}
int getBaudrate() {
  return getFromPersistent(BAUDRATE_EEPROM_ADDR, DEFAULT_BAUDRATE);
}
