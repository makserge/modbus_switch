#include <ModbusSlave.h>
#include <Wire.h>
#include <OneWire.h>
#include <OneButton.h>
#include <PCF8574.h>
#include <IWatchdog.h>

#define SLAVE_ID 32
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA10
#define RS485_TX_PIN PA9
#define RS485_TX_ENABLE_PIN PA11

#define INPUT1_PIN PA3
#define INPUT2_PIN PA4
#define INPUT3_PIN PA6
#define INPUT4_PIN PB0
#define INPUT5_PIN PB1

#define OUTPUT1_PIN PA12
#define OUTPUT2_PIN PA15
#define OUTPUT3_PIN PB3
#define OUTPUT4_PIN PB4
#define OUTPUT5_PIN PB5

#define I2C_SDA PB7
#define I2C_SCL PB6

#define DS18B20_PIN PA8
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

#define PCF8574_I2C_ADDRESS 0x38

#define PCF_INPUT1_PIN P0
#define PCF_INPUT2_PIN P1
#define PCF_INPUT3_PIN P2
#define PCF_INPUT4_PIN P3

#define PCF_OUTPUT1_PIN P4
#define PCF_OUTPUT2_PIN P5
#define PCF_OUTPUT3_PIN P6
#define PCF_OUTPUT4_PIN P7

const uint8_t OUT1 = 0;
const uint8_t OUT2 = 1;
const uint8_t OUT3 = 2;
const uint8_t OUT4 = 3;
const uint8_t OUT5 = 4;

const uint8_t TOILET_TEMP = 0;
const uint8_t BATHROOM_TEMP = 1;
const uint8_t BATHROOM_HUMIDITY = 2;

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t INPUT_COUNT = 3;

uint8_t outputState[5] = { LOW, LOW, LOW, LOW, LOW }; //{ OUT1, OUT2, OUT3, OUT4, OUT5 }
uint16_t inputRegister[INPUT_COUNT] = { 0, 0, 0 }; //{ TOILET_TEMP, BATHROOM_TEMP, BATHROOM_HUMIDITY }

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);
OneButton button2(INPUT2_PIN, true, false);
OneButton button3(INPUT3_PIN, true, false);
OneButton button4(INPUT4_PIN, true, false);
OneButton button5(INPUT5_PIN, true, false);
HardwareTimer *timer1;
OneWire ds(DS18B20_PIN);
PCF8574 pcf8574(PCF8574_I2C_ADDRESS);

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(OUT1, outputState[OUT1]);
  slave.writeCoilToBuffer(OUT2, outputState[OUT2]);
  slave.writeCoilToBuffer(OUT3, outputState[OUT3]);
  slave.writeCoilToBuffer(OUT4, outputState[OUT4]);
  slave.writeCoilToBuffer(OUT5, outputState[OUT5]);
  return STATUS_OK;
}

void setOutput(uint8_t out) {
  uint8_t pin;
  uint8_t pcfPin;
  uint8_t value;
  switch (out) {
    case OUT1:
      pin = OUTPUT1_PIN;
      pcfPin = PCF_OUTPUT1_PIN;
      break;
    case OUT2:
      pin = OUTPUT2_PIN;
      pcfPin = PCF_OUTPUT2_PIN;
      break;
    case OUT3:
      pin = OUTPUT3_PIN;
      pcfPin = PCF_OUTPUT3_PIN;
      break;
    case OUT4:
      pin = OUTPUT4_PIN;
      pcfPin = PCF_OUTPUT4_PIN;
      break;
    case OUT5:
      pin = OUTPUT5_PIN;
      pcfPin = 254;
      break;        
  }
  value = outputState[out];
  digitalWrite(pin, value);
  if (pcfPin != 254) {
    pcf8574.digitalWrite(pcfPin, value);
  }  
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastOutputState[5] = { outputState[OUT1], outputState[OUT2], outputState[OUT3], outputState[OUT4], outputState[OUT5] };
  
  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  for (uint8_t i = 0; i < 5; i++) {
    if (outputState[i] != lastOutputState[i]) {
      setOutput(i);
    }
  }
  
  return STATUS_OK;
}

void clickButton1() {
  outputState[OUT1] = !outputState[OUT1];
  setOutput(OUT1);
}

void clickButton2() {
  outputState[OUT2] = !outputState[OUT2];
  setOutput(OUT2);
}

void clickButton3() {
  outputState[OUT3] = !outputState[OUT3];
  setOutput(OUT3);
}

void clickButton4() {
  outputState[OUT4] = !outputState[OUT4];
  setOutput(OUT4);
}

void clickButton5() {
  outputState[OUT5] = !outputState[OUT5];
  setOutput(OUT5);
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
  button5.attachClick(clickButton5);
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
  inputRegister[TOILET_TEMP] = readDS() * 10;
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

void initPCF8574() {
  pcf8574.pinMode(PCF_INPUT1_PIN, INPUT);
  pcf8574.pinMode(PCF_INPUT2_PIN, INPUT);
  pcf8574.pinMode(PCF_INPUT3_PIN, INPUT);
  pcf8574.pinMode(PCF_INPUT4_PIN, INPUT);
  pcf8574.pinMode(PCF_OUTPUT1_PIN, OUTPUT);
  pcf8574.pinMode(PCF_OUTPUT2_PIN, OUTPUT);
  pcf8574.pinMode(PCF_OUTPUT3_PIN, OUTPUT);
  pcf8574.pinMode(PCF_OUTPUT4_PIN, OUTPUT);

  if (pcf8574.begin()) {
    pcf8574.digitalWrite(PCF_OUTPUT1_PIN, LOW);
    pcf8574.digitalWrite(PCF_OUTPUT2_PIN, LOW);
    pcf8574.digitalWrite(PCF_OUTPUT3_PIN, LOW);
    pcf8574.digitalWrite(PCF_OUTPUT4_PIN, LOW);
  }
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
  initButtons();
  initI2C();
  initHTU21D();
  initDS();
  initPCF8574();
  initPeriodicalTimer();
  
  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = readInputRegister;
  
  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);

  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  button1.tick(pcf8574.digitalRead(PCF_INPUT1_PIN) == LOW);
  button2.tick(pcf8574.digitalRead(PCF_INPUT2_PIN) == LOW);
  button3.tick(pcf8574.digitalRead(PCF_INPUT3_PIN) == LOW);
  button4.tick(pcf8574.digitalRead(PCF_INPUT4_PIN) == LOW);
  button5.tick();
  
  slave.poll();
  
  IWatchdog.reload();
}
