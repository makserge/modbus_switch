#include <ModbusSlave.h>
#include <Wire.h>
#include <OneButton.h>

#define SLAVE_ID 21
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA3
#define RS485_TX_PIN PA2
#define RS485_TX_ENABLE_PIN PB1

#define INPUT1_PIN PA4
#define INPUT2_PIN PA0

#define OUTPUT1_PIN PA5
#define OUTPUT2_PIN PA6

#define I2C_SDA PA10
#define I2C_SCL PA9

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

const uint8_t AIR_TEMP = 0;
const uint8_t HUMIDITY = 1;

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ
const uint16_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t INPUT_REGISTER_COUNT = 2;

uint8_t outputState[2] = { LOW, LOW }; //{ OUT1_STATE, OUT2_STATE }
uint16_t inputRegister[INPUT_REGISTER_COUNT] = { 0, 0 }; //{ AIR_TEMP, HUMIDITY }

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);
OneButton button2(INPUT2_PIN, true, false);
HardwareTimer *timer1;

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(OUT1_STATE, outputState[OUT1_STATE]);
  slave.writeCoilToBuffer(OUT2_STATE, outputState[OUT2_STATE]);
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

  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  if (outputState[OUT1_STATE] != lastOutputState1) {
    setOutput(OUTPUT1_PIN, outputState[OUT1_STATE]);
  }
  if (outputState[OUT2_STATE] != lastOutputState2) {
    setOutput(OUTPUT2_PIN, outputState[OUT2_STATE]);
  } 
  return STATUS_OK;
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

void updateSensors() {
  float airTemp = readTemperature();
  inputRegister[AIR_TEMP] = airTemp * 10; 
  inputRegister[HUMIDITY] = readCompensatedHumidity(airTemp);
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
  for (uint16_t i = 0; i < INPUT_REGISTER_COUNT; i++) {
    slave.writeRegisterToBuffer(i, inputRegister[i]);
  }
  return STATUS_OK;
}

void initButtons() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);

  button1.attachClick(clickButton1);
  button2.attachClick(clickButton2);
}

void clickButton1() {
  outputState[OUT1_STATE] = !outputState[OUT1_STATE];
  setOutput(OUTPUT1_PIN, outputState[OUT1_STATE]);
}

void clickButton2() {
  outputState[OUT2_STATE] = !outputState[OUT2_STATE];
  setOutput(OUTPUT2_PIN, outputState[OUT2_STATE]);
}

void setup() {   
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
  slave.poll();
}
