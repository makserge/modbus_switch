#include <ModbusSlave.h>
#include <Wire.h>
#include <OneWire.h>
#include <OneButton.h>
#include <IWatchdog.h>

#define SLAVE_ID 32
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA10
#define RS485_TX_PIN PA9
#define RS485_TX_ENABLE_PIN PA12

#define INPUT1_PIN PA2
#define INPUT2_PIN PA3
#define INPUT3_PIN PA4
#define INPUT4_PIN PA5
#define INPUT5_PIN PA6

#define OUTPUT1_PIN PB0
#define OUTPUT2_PIN PB1
#define OUTPUT3_PIN PB3
#define OUTPUT4_PIN PB4
#define OUTPUT5_PIN PB5

#define I2C_SDA PB7
#define I2C_SCL PB6

#define DS18B20_PIN PA15

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
const uint8_t OUT3_STATE = 2;
const uint8_t OUT4_STATE = 3;
const uint8_t OUT5_STATE = 4;

const uint8_t TOILET_TEMP = 0;
const uint8_t BATHROOM_TEMP = 1;
const uint8_t BATHROOM_HUMIDITY = 2;

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ
const uint16_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t INPUT_COUNT = 3;

uint8_t outputState[5] = { LOW, LOW, LOW, LOW, LOW }; //{ OUT1_STATE, OUT2_STATE, OUT3_STATE, OUT4_STATE, OUT5_STATE }
uint16_t inputRegister[INPUT_COUNT] = { 0, 0, 0 }; //{ TOILET_TEMP, BATHROOM_TEMP, BATHROOM_HUMIDITY }

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);
OneButton button2(INPUT2_PIN, true, false);
OneButton button3(INPUT3_PIN, true, false);
OneButton button4(INPUT4_PIN, true, false);
OneButton button5(INPUT5_PIN, true, false);
HardwareTimer *timer1;
OneWire ds(DS18B20_PIN);

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(OUT1_STATE, outputState[OUT1_STATE]);
  slave.writeCoilToBuffer(OUT2_STATE, outputState[OUT2_STATE]);
  slave.writeCoilToBuffer(OUT3_STATE, outputState[OUT3_STATE]);
  slave.writeCoilToBuffer(OUT4_STATE, outputState[OUT4_STATE]);
  slave.writeCoilToBuffer(OUT5_STATE, outputState[OUT5_STATE]);
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
  uint8_t lastOutputState3 = outputState[OUT3_STATE];
  uint8_t lastOutputState4 = outputState[OUT4_STATE];
  uint8_t lastOutputState5 = outputState[OUT5_STATE];
  
  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  if (outputState[OUT1_STATE] != lastOutputState1) {
    setOutput(OUTPUT1_PIN, outputState[OUT1_STATE]);
  }
  if (outputState[OUT2_STATE] != lastOutputState2) {
    setOutput(OUTPUT2_PIN, outputState[OUT2_STATE]);
  }
  if (outputState[OUT3_STATE] != lastOutputState3) {
    setOutput(OUTPUT3_PIN, outputState[OUT3_STATE]);
  }
  if (outputState[OUT4_STATE] != lastOutputState4) {
    setOutput(OUTPUT4_PIN, outputState[OUT4_STATE]);
  }
  if (outputState[OUT5_STATE] != lastOutputState5) {
    setOutput(OUTPUT5_PIN, outputState[OUT5_STATE]);
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

void clickButton3() {
  outputState[OUT3_STATE] = !outputState[OUT3_STATE];
  setOutput(OUTPUT3_PIN, outputState[OUT3_STATE]);
}

void clickButton4() {
  outputState[OUT4_STATE] = !outputState[OUT4_STATE];
  setOutput(OUTPUT4_PIN, outputState[OUT4_STATE]);
}

void longPressStart() {
  outputState[OUT5_STATE] = HIGH;
}

void longPressStop() {
  outputState[OUT5_STATE] = LOW;
}

void initButtons() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(INPUT3_PIN, INPUT);
  pinMode(INPUT4_PIN, INPUT);
  pinMode(INPUT5_PIN, INPUT);
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);
  pinMode(OUTPUT3_PIN, OUTPUT);
  pinMode(OUTPUT4_PIN, OUTPUT);
  pinMode(OUTPUT5_PIN, OUTPUT);
  
  button1.attachClick(clickButton1);
  button2.attachClick(clickButton2);
  button3.attachClick(clickButton3);
  button4.attachClick(clickButton4);
  button5.attachLongPressStart(longPressStart);
  button5.attachLongPressStart(longPressStop);
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

float readDS18B20() {
  uint8_t typeS;
  uint8_t data[12];
  uint8_t address[8];
  
  if (!ds.search(address)) {
    ds.reset_search();
    delay(250);
    return 0;
  }
  if (OneWire::crc8(address, 7) != address[7]) {
    return 0;
  }
  switch (address[0]) {
    case 0x10:
      typeS = 1;
      break;
    case 0x28:
    case 0x22:
      typeS = 0;
      break;
    default:
      return 0;
  } 
  ds.reset();
  ds.select(address);
  ds.write(0x44, 1);
  delay(1000);
  ds.reset();
  ds.select(address);    
  ds.write(0xBE);

  for (uint8_t i = 0; i < 9; i++) {
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (typeS) {
    raw = raw << 3;
    if (data[7] == 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) {
      raw = raw & ~7;
    }
    else if (cfg == 0x20) {
      raw = raw & ~3;
    }
    else if (cfg == 0x40) {
      raw = raw & ~1;
    }
  }
  return (float)raw / 16.0;
}

void updateSensors() {
  inputRegister[TOILET_TEMP] = readDS18B20() * 10;
  float bathroomTemp = readTemperature();
  inputRegister[BATHROOM_TEMP] = bathroomTemp * 10; 
  inputRegister[BATHROOM_HUMIDITY] = readCompensatedHumidity(bathroomTemp);
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
 * Handle Read Input Registers (FC=04)
 */
uint8_t readInputRegister(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < INPUT_COUNT; i++) {
    slave.writeRegisterToBuffer(i, inputRegister[i]);
  }
  return STATUS_OK;
}

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM1);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, HERTZ_FORMAT);
  timer->attachInterrupt(updateSensors);
  timer->resume();
}

void setup() {
  IWatchdog.begin(WATCHDOG_TIMEOUT);
  
  initButtons();
  initPeriodicalTimer();
  initI2C();
  initHTU21D();
  
  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = readInputRegister;
  
  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);
}

void loop() {
  button1.tick();
  button2.tick();
  button3.tick();
  button4.tick();
  button5.tick();
  slave.poll();
  IWatchdog.reload();
}
