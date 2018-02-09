#include <EEPROM.h>
#include <SimpleModbusSlave.h>
#include <SoftwareSerial.h>

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
#define TX_PIN 2

//2 analog input
#define INPUTS_COUNT 2

//2 TTL outputs
#define TTL_OUTPUTS_COUNT 2

#define DUMPERS_COUNT 3

//Analog ouptuts has this value by default
#define DEFAULT_DUMPER_POSITION 0
#define DEFAULT_ANALOG_IN_VAL 0
#define DEFAULT_TTL_OUT_VAL 0

#define DEFAULT_PROPORTIONAL_K 10
#define DEFAULT_CN2A_OUT_OVERRIDE_VAL -1
#define DEFAULT_CN2A_MIN_TRESHOLD 20

#define STATUS_LED A2
#define RESET_EEPROM_BTN A5

//CN2A is mitsubishi interface to control AC airflow (1-10out)
#define CN2A_PIN 10

byte CURRENT_SLAVE_ID = DEFAULT_SLAVE_ID;
byte CURRENT_BAUDRATE = DEFAULT_BAUDRATE;

byte CURRENT_PROPORTIONAL_K = DEFAULT_PROPORTIONAL_K;
byte CURRENT_CN2A_MIN_TRESHOLD = DEFAULT_CN2A_MIN_TRESHOLD;

uint8_t analogInPins[INPUTS_COUNT] = { A0, A1 };
uint8_t dumpersPins[DUMPERS_COUNT] = { 5, 6, 9 };

uint8_t ttlOutPins[TTL_OUTPUTS_COUNT] = { 7, 8 };

//Holding registers
enum
{
  //dumpers
  DUMPER_POSITION_REG_1,
  DUMPER_POSITION_REG_2,
  DUMPER_POSITION_REG_3,

  ANALOG_IN_REG_1,
  ANALOG_IN_REG_2,

  TTL_OUT_REG_0,
  TTL_OUT_REG_1,

  PROPORTIONAL_K_REG,
  CN2A_MIN_TRESHOLD_REG,
  CN2A_OUT_OVERRIDE_REG,
  
  //settings
  BAUDRATE_REG,
  SLAVE_ID_REG,

  HOLDING_REGS_SIZE
};

// EEPROM adresses
enum {
  SLAVE_ID_EEPROM_ADDR,
  BAUDRATE_EEPROM_ADDR,
  PROPORTINAL_K_EEPROM_ADDR,
  CN2A_MIN_TRESHOLD_EEPROM_ADDR
};

// data array for modbus network sharing
unsigned int holdingRegs[HOLDING_REGS_SIZE] = {
  //1-10 out
  DEFAULT_DUMPER_POSITION,
  DEFAULT_DUMPER_POSITION,
  DEFAULT_DUMPER_POSITION,

  DEFAULT_ANALOG_IN_VAL,
  DEFAULT_ANALOG_IN_VAL,

  DEFAULT_TTL_OUT_VAL,
  DEFAULT_TTL_OUT_VAL,

  DEFAULT_PROPORTIONAL_K,
  DEFAULT_CN2A_MIN_TRESHOLD,
  DEFAULT_CN2A_OUT_OVERRIDE_VAL,
  
  CURRENT_BAUDRATE,
  CURRENT_SLAVE_ID,
};

SoftwareSerial softSerial(A4, A3); // RX, TX

void setup() {
  softSerial.begin(9600);
  
  // press reset MC and clear eeprom buttons together
  if (digitalRead(RESET_EEPROM_BTN) == HIGH) {
   softSerial.println("EEPROM Reset BTN pushed. Reset EEPROM...");
   resetEeprom();
  }
  
  pinMode(STATUS_LED, OUTPUT);
  
  for (int i = 0; i < TTL_OUTPUTS_COUNT; i++)
  {
     pinMode(ttlOutPins[i], OUTPUT);
  }
  
  startModbus();
  initAirflowSettings();
}

void startModbus(){
  initModbusSettings();
  
  softSerial.println("Modbus Slave Device Start");

  softSerial.print("Baudrate: ");
  softSerial.print(BAUDRATES[CURRENT_BAUDRATE]);
  softSerial.print(" Slave ID: ");
  softSerial.print(CURRENT_SLAVE_ID);
  softSerial.print(" Registers count: ");
  softSerial.println(HOLDING_REGS_SIZE);

  modbus_configure(&Serial, BAUDRATES[CURRENT_BAUDRATE], SERIAL_8N2, CURRENT_SLAVE_ID, TX_PIN, HOLDING_REGS_SIZE, holdingRegs);
}


void loop() {
  statusBlink();
  modbus_update();

  int dumpersSumm = 0;
  
  //dumpers
  for (int i = 0; i < DUMPERS_COUNT; i++)
  {
    uint8_t value = holdingRegs[i] = constrain(holdingRegs[i], 0, 100); //restrict range between 0 and 100
    dumpersSumm += 100 - value;
    int pwmValue = map(holdingRegs[i], 0, 100, 0, 250); //convert to pwm (tuned to certain hardware to output exactly 10 volts)
    analogWrite(dumpersPins[i], pwmValue);
  }
  
  uint8_t airflowValue;
  
  if (holdingRegs[CN2A_OUT_OVERRIDE_REG] != -1) {
    airflowValue = holdingRegs[CN2A_OUT_OVERRIDE_REG] = constrain(holdingRegs[CN2A_OUT_OVERRIDE_REG], 0, 100);
  } else {
    //automatic airflow control (proportially increase airflow when dumpers opens)
    airflowValue = constrain(CURRENT_CN2A_MIN_TRESHOLD + (dumpersSumm * (float(CURRENT_PROPORTIONAL_K) / 100)), 0, 100);
  }
  
  analogWrite(CN2A_PIN, map(airflowValue, 0, 100, 0, 255));

  // 1-10 inputs
  for (int i = 0; i < INPUTS_COUNT; i++)
  {
     holdingRegs[DUMPERS_COUNT + i] = analogRead(analogInPins[i]);
  }

  for (int i = 0; i < TTL_OUTPUTS_COUNT; i++)
  {
     int value = holdingRegs[DUMPERS_COUNT + INPUTS_COUNT + i];
     value = holdingRegs[DUMPERS_COUNT + INPUTS_COUNT + i] = value > 0 ? 1 : 0;
     
     digitalWrite(ttlOutPins[i], value);
  }
  
  watchAirflowSettings();
  watchModbusSettings();
}

unsigned long previousLedBlinkMillis = 0;
int statusLedState = LOW;  

void statusBlink() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousLedBlinkMillis >= 1000) {
    previousLedBlinkMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (statusLedState == LOW) {
      statusLedState = HIGH;
    } else {
      statusLedState = LOW;
    }
    
    digitalWrite(STATUS_LED, statusLedState);
  }
}

void initAirflowSettings() {
  CURRENT_PROPORTIONAL_K = getFromPersistent(PROPORTINAL_K_EEPROM_ADDR, DEFAULT_PROPORTIONAL_K);
  CURRENT_CN2A_MIN_TRESHOLD = getFromPersistent(CN2A_MIN_TRESHOLD_EEPROM_ADDR, DEFAULT_CN2A_MIN_TRESHOLD);

  holdingRegs[PROPORTIONAL_K_REG] = CURRENT_PROPORTIONAL_K;
  holdingRegs[CN2A_MIN_TRESHOLD_REG] = CURRENT_CN2A_MIN_TRESHOLD;
}

void watchAirflowSettings() {
  if (holdingRegs[PROPORTIONAL_K_REG] != CURRENT_PROPORTIONAL_K || holdingRegs[CN2A_MIN_TRESHOLD_REG] != CURRENT_CN2A_MIN_TRESHOLD) {
    CURRENT_PROPORTIONAL_K = holdingRegs[PROPORTIONAL_K_REG] = constrain(holdingRegs[PROPORTIONAL_K_REG], 1, 100);
    CURRENT_CN2A_MIN_TRESHOLD = holdingRegs[CN2A_MIN_TRESHOLD_REG] = constrain(holdingRegs[CN2A_MIN_TRESHOLD_REG], 1, 100);
    
    saveToPersistent(PROPORTINAL_K_EEPROM_ADDR, CURRENT_PROPORTIONAL_K);
    saveToPersistent(CN2A_MIN_TRESHOLD_EEPROM_ADDR, CURRENT_CN2A_MIN_TRESHOLD);
  }
}

void initModbusSettings() {
  CURRENT_BAUDRATE = getFromPersistent(BAUDRATE_EEPROM_ADDR, DEFAULT_BAUDRATE);
  CURRENT_SLAVE_ID = getFromPersistent(SLAVE_ID_EEPROM_ADDR, DEFAULT_SLAVE_ID);

  holdingRegs[BAUDRATE_REG] = CURRENT_BAUDRATE;
  holdingRegs[SLAVE_ID_REG] = CURRENT_SLAVE_ID;
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

void saveModbusSettings(int baudrate, int id) {
   //save only if baudare inside of available range
  if (baudrate < BAUDRATES_SIZE) {
    CURRENT_BAUDRATE = baudrate;
    saveToPersistent(BAUDRATE_EEPROM_ADDR, baudrate);
  }
  
  saveToPersistent(SLAVE_ID_EEPROM_ADDR, id);
  CURRENT_SLAVE_ID = id;
}


byte getFromPersistent(int address, byte substitute) {
  byte value = EEPROM.read(address);
  return value == 255 ? substitute : value;
}

void saveToPersistent(int address, byte value) {
  EEPROM.update(address, value);
}

void resetEeprom(){
for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 255);
  }  
}
