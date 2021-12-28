#include <ModbusSlave.h>
#include <Wire.h>
#include <OneWire.h>
#include <OneButton.h>
#include <IWatchdog.h>
#include <EEPROM.h>

#define SLAVE_ID 31
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA10
#define RS485_TX_PIN PA9
#define RS485_TX_ENABLE_PIN PA12

#define INPUT1_PIN PA4
#define INPUT2_PIN PA5
#define THERMOSTAT_ON_PIN PA6

#define OUTPUT1_PIN PB0
#define OUTPUT2_PIN PB1
#define THERMOSTAT_LED_PIN PB5
#define THERMOSTAT_OUTPUT_PIN PA7

#define I2C_SDA PB7
#define I2C_SCL PB6

#define DS18B20_PIN PA15
#define DS18B20_PRECISION 11

#define HTU21D_I2C_ADDRESS 0x40
#define HTU21D_RES_RH8_TEMP12 0x01
#define HTU21D_HEATER_ON 0x04
#define HTU21D_HEATER_OFF 0xFB
#define HTU21D_USER_REGISTER_WRITE 0xE6
#define HTU21D_USER_REGISTER_READ 0xE7
#define HTU21D_TRIGGER_TEMP_MEASURE_HOLD 0xE3
#define HTU21D_CRC8_POLYNOMINAL 0x13100
#define HTU21D_TEMP_COEFFICIENT -0.15
#define HTU21D_TRIGGER_HUMD_MEASURE_HOLD 0xE5

const uint8_t OUT1_STATE = 0;
const uint8_t OUT2_STATE = 1;
const uint8_t THERMOSTAT_STATE = 2;

const uint8_t FLOOR_TEMP = 0;
const uint8_t AIR_TEMP = 1;
const uint8_t HUMIDITY = 2;

const uint8_t THERMOSTAT_ON = 0;
const uint8_t THERMOSTAT_TEMP = 1;
const uint8_t MIN_FLOOR_TEMP = 2;
const uint8_t MAX_FLOOR_TEMP = 3;
const uint8_t HYST_TEMP = 4;

const uint8_t THERMOSTAT_ON_EEPROM = 10;
const uint8_t THERMOSTAT_TEMP_EEPROM = 20;
const uint8_t MIN_FLOOR_TEMP_EEPROM = 30;
const uint8_t MAX_FLOOR_TEMP_EEPROM = 40;
const uint8_t HYST_TEMP_EEPROM = 50;

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t HOLDING_COUNT = 5;

uint8_t outputState[3] = { LOW, LOW, LOW }; //{ OUT1_STATE, OUT2_STATE, THERMOSTAT_STATE }
uint16_t inputRegister[3] = { 0, 0, 0 }; //{ FLOOR_TEMP, AIR_TEMP, HUMIDITY }
uint16_t holdingRegister[HOLDING_COUNT] = { 0, 25, 20, 40, 1 }; //{ THERMOSTAT_ON, THERMOSTAT_TEMP, MIN_FLOOR_TEMP, MAX_FLOOR_TEMP, HYST_TEMP }

float floorTemp, airTemp;
uint8_t ledStatus = LOW;

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);
OneButton button2(INPUT2_PIN, true, false);
OneButton thermostatButton(THERMOSTAT_ON_PIN, true, false);
OneWire ds(DS18B20_PIN);

// Handle the function code Read Input Registers (FC=04) and write back the values from analog input pins (input registers).
uint8_t readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    for (int i = 0; i < length; i++) {
        slave.writeRegisterToBuffer(i, inputRegister[i + address]);
    }
    return STATUS_OK;
}

void updateLed() {
  if (outputState[THERMOSTAT_STATE]) {
    ledStatus = !ledStatus;   
  } else {
    ledStatus = holdingRegister[THERMOSTAT_ON];
  }
  setOutput(THERMOSTAT_LED_PIN, ledStatus);
}

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  for (int i = 0; i < length; i++) {
    slave.writeCoilToBuffer(i, outputState[i + address]);
  }
  
  return STATUS_OK;
}

void setOutput(uint8_t pin, uint8_t value) {
  digitalWrite(pin, value);
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastOutputState1 = outputState[OUT1_STATE];
  uint8_t lastOutputState2 = outputState[OUT2_STATE];
  uint8_t lastThermostatState = outputState[THERMOSTAT_STATE];

  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  if (outputState[OUT1_STATE] != lastOutputState1) {
    setOutput(OUTPUT1_PIN, outputState[OUT1_STATE]);
  }
  if (outputState[OUT2_STATE] != lastOutputState2) {
    setOutput(OUTPUT2_PIN, outputState[OUT2_STATE]);
  }
  if (outputState[THERMOSTAT_STATE] != lastThermostatState) {
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  } 
  return STATUS_OK;
}

void clickButton1() {
  outputState[OUT1_STATE] = !outputState[OUT1_STATE];
  setOutput(OUTPUT1_PIN, outputState[OUT1_STATE]);
}

void clickButton2() {
  outputState[OUT2_STATE] = !outputState[OUT2_STATE];
  setOutput(OUTPUT2_PIN, outputState[OUT2_STATE]);
}

void clickThermostatButton() {
  holdingRegister[THERMOSTAT_ON] = !holdingRegister[THERMOSTAT_ON];
  setOutput(THERMOSTAT_LED_PIN, holdingRegister[THERMOSTAT_ON]);
  if (!holdingRegister[THERMOSTAT_ON]) {
    outputState[THERMOSTAT_STATE] = LOW;
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  }
  EEPROM.put(THERMOSTAT_ON_EEPROM, holdingRegister[THERMOSTAT_ON]); 
}

void loadData() {
  EEPROM.get(THERMOSTAT_ON_EEPROM, holdingRegister[THERMOSTAT_ON]);
  if (holdingRegister[THERMOSTAT_ON] > 1) {
    EEPROM.put(THERMOSTAT_ON_EEPROM, 0);
    EEPROM.put(THERMOSTAT_TEMP_EEPROM, holdingRegister[THERMOSTAT_TEMP]);
    EEPROM.put(MIN_FLOOR_TEMP_EEPROM, holdingRegister[MIN_FLOOR_TEMP]);
    EEPROM.put(MAX_FLOOR_TEMP_EEPROM, holdingRegister[MAX_FLOOR_TEMP]);
    EEPROM.put(HYST_TEMP_EEPROM, holdingRegister[HYST_TEMP]);
  }
  EEPROM.get(THERMOSTAT_ON_EEPROM, holdingRegister[THERMOSTAT_ON]);
  EEPROM.get(THERMOSTAT_TEMP_EEPROM, holdingRegister[THERMOSTAT_TEMP]);
  EEPROM.get(MIN_FLOOR_TEMP_EEPROM, holdingRegister[MIN_FLOOR_TEMP]);
  EEPROM.get(MAX_FLOOR_TEMP_EEPROM, holdingRegister[MAX_FLOOR_TEMP]);
  EEPROM.get(HYST_TEMP_EEPROM, holdingRegister[HYST_TEMP]);
}

void initButtons() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(THERMOSTAT_ON_PIN, INPUT);
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);
  pinMode(THERMOSTAT_LED_PIN, OUTPUT);
  pinMode(THERMOSTAT_OUTPUT_PIN, OUTPUT);

  button1.attachClick(clickButton1);
  button2.attachClick(clickButton2);
  thermostatButton.attachClick(clickThermostatButton);
}

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM1);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, HERTZ_FORMAT);
  timer->attachInterrupt(updateSensors);
  timer->resume();
}

float readTemperature() {
  int8_t qntRequest = 3;
  uint16_t rawTemperature = 0;
  uint8_t checksum = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_TEMP_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) {
    return I2C_ERROR;
  }

  delay(22);

  Wire.requestFrom(HTU21D_I2C_ADDRESS, qntRequest);
  if (Wire.available() != qntRequest) {
    return I2C_ERROR;
  }
  rawTemperature = Wire.read() << 8;
  rawTemperature |= Wire.read();
  checksum = Wire.read();

  if (checkCRC8(rawTemperature) != checksum) {
    return I2C_ERROR;
  }
  return (0.002681 * (float)rawTemperature - 46.85);
}

uint8_t checkCRC8(uint16_t data) {
  for (uint8_t bit = 0; bit < 16; bit++) {
    if (data & 0x8000) {
      data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
    } else {
      data <<= 1;
    }
  }
  return data >>= 8;
}

float readHumidity() {
  uint16_t rawHumidity = 0;
  uint8_t checksum = 0;
  float humidity = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_HUMD_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) {
    return I2C_ERROR;
  }
  delay(4);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 3);
  if (Wire.available() != 3) {
    return I2C_ERROR;
  }

  rawHumidity = Wire.read() << 8;
  rawHumidity |= Wire.read();
  checksum = Wire.read();

  if (checkCRC8(rawHumidity) != checksum) {
    return I2C_ERROR;
  }

  rawHumidity ^= 0x02;
  humidity = (0.001907 * (float)rawHumidity - 6);

  if (humidity < 0) {
    humidity = 0;
  } else if (humidity > 100) {
    humidity = 100;
  }
  return humidity;
}

float readCompensatedHumidity(float temperature) {
  float humidity = readHumidity();
  if (humidity == I2C_ERROR || temperature == I2C_ERROR) {
    return I2C_ERROR;
  }
  if (temperature > 0 && temperature < 80) {
    humidity = humidity + (25.0 - temperature) * HTU21D_TEMP_COEFFICIENT;
  }
  return humidity;
}

void initDS() {
  ds.reset();
  ds.write(0xCC);
  ds.write(0x4E);
  ds.write(0);
  ds.write(0);
  ds.write(DS18B20_PRECISION << 5);
  ds.write(0x48);
}

float readDS() {
  uint8_t data[2];

  ds.reset();
  ds.write(0xCC);
  ds.write(0xBE);
  data[0] = ds.read();
  data[1] = ds.read();

  ds.reset();
  ds.write(0xCC);
  ds.write(0x44);

  int16_t raw = (data[1] << 8) | data[0];
  return (float)raw / 16.0;
}

void updateSensors() {
  airTemp = readTemperature();
  inputRegister[AIR_TEMP] = airTemp * 10;
  inputRegister[HUMIDITY] = readCompensatedHumidity(airTemp);
  
  floorTemp = readDS();
  
  if (floorTemp == 85) {//initial value
    return;
  }
  inputRegister[FLOOR_TEMP] = floorTemp * 10;
 
  if (holdingRegister[THERMOSTAT_ON]) {
    if (outputState[THERMOSTAT_STATE]) {
      if ((floorTemp >= holdingRegister[MAX_FLOOR_TEMP]) || (airTemp >= holdingRegister[THERMOSTAT_TEMP])) {
        outputState[THERMOSTAT_STATE] = 0;
        setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
      }  
    } else {  
      if (airTemp <= holdingRegister[THERMOSTAT_TEMP] - holdingRegister[HYST_TEMP]) {
        outputState[THERMOSTAT_STATE] = 1;
        setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
      }
    }
  }
  updateLed();
}
 
void initI2C() {
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
}

void initHTU21D() {
  uint8_t userRegisterData = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_READ);
  Wire.endTransmission(true);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 1);
  if (Wire.available() != 1) {
    return;
  }
  userRegisterData = Wire.read();

  userRegisterData &= 0x7E;
  userRegisterData |= HTU21D_RES_RH8_TEMP12;
  userRegisterData &= HTU21D_HEATER_OFF;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_WRITE);
  Wire.write(userRegisterData);
  Wire.endTransmission(true);
}

/**
 * Handle Read Holding Registers (FC=03)
 */
uint8_t readHolding(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    slave.writeRegisterToBuffer(i, holdingRegister[i + address]);
  }
  return STATUS_OK;
}

/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 */
uint8_t writeHolding(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    holdingRegister[i + address] = slave.readRegisterFromBuffer(i);
  }
  if (holdingRegister[THERMOSTAT_TEMP] > holdingRegister[MAX_FLOOR_TEMP]) {
    holdingRegister[THERMOSTAT_TEMP] = holdingRegister[MAX_FLOOR_TEMP];
  } else if (holdingRegister[THERMOSTAT_TEMP] < holdingRegister[MIN_FLOOR_TEMP]) {
    holdingRegister[THERMOSTAT_TEMP] = holdingRegister[MIN_FLOOR_TEMP];
  }
  if (!holdingRegister[THERMOSTAT_ON]) {
    outputState[THERMOSTAT_STATE] = LOW;
    setOutput(THERMOSTAT_OUTPUT_PIN, outputState[THERMOSTAT_STATE]);
  }

  EEPROM.put(THERMOSTAT_ON_EEPROM, holdingRegister[THERMOSTAT_ON]);
  EEPROM.put(THERMOSTAT_TEMP_EEPROM, holdingRegister[THERMOSTAT_TEMP]);
  EEPROM.put(MIN_FLOOR_TEMP_EEPROM, holdingRegister[MIN_FLOOR_TEMP]);
  EEPROM.put(MAX_FLOOR_TEMP_EEPROM, holdingRegister[MAX_FLOOR_TEMP]);
  EEPROM.put(HYST_TEMP_EEPROM, holdingRegister[HYST_TEMP]);
  
  return STATUS_OK;
}

void setup() {
  loadData();
  initButtons();
  initI2C();
  initHTU21D();
  initDS();
  initPeriodicalTimer();

  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = readAnalogIn;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readHolding;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeHolding;

  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);

  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  button1.tick();
  button2.tick();
  thermostatButton.tick();
  slave.poll();
  IWatchdog.reload();
}
