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
#define INPUT5_PIN PB1//Toilet presence sensor

#define BATH_FAN_OUTPUT_PIN PA12
#define TOILET_FAN_OUTPUT_PIN PA15
#define BATH_LIGHT_OUTPUT_PIN PB3
#define TOILET_LIGHT_OUTPUT_PIN PB4
#define BOILER_OUTPUT_PIN PB5
#define HEAT1_OUTPUT_PIN PA0
#define HEAT2_OUTPUT_PIN PA1
#define HEAT3_OUTPUT_PIN PA2

#define BATH_LIGHT_LEVEL_PIN PA7
#define TOILET_LIGHT_LEVEL_PIN PA5

#define PCF_BATH_FAN_INPUT_PIN P0
#define PCF_TOILET_FAN_INPUT_PIN P1
#define PCF_BATH_LIGHT_INPUT_PIN P2
#define PCF_TOILET_LIGHT_INPUT_PIN P3

#define PCF_BATH_FAN_OUTPUT_PIN P7
#define PCF_TOILET_FAN_OUTPUT_PIN P6
#define PCF_BATH_LIGHT_OUTPUT_PIN P5
#define PCF_TOILET_LIGHT_OUTPUT_PIN P4

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

const uint8_t BATH_FAN = 0;
const uint8_t TOILET_FAN = 1;
const uint8_t BATH_LIGHT = 2;
const uint8_t TOILET_LIGHT = 3;
const uint8_t BOILER = 4;
const uint8_t HEAT1 = 5;
const uint8_t HEAT2 = 6;
const uint8_t HEAT3 = 7;

const uint8_t BATH_LIGHT_LEVEL = 0;
const uint8_t BATH_FAN_AUTO = 1;
const uint8_t BATH_FAN_TOLERANCE = 2;
const uint8_t TOILET_FAN_AUTO = 3;
const uint8_t TOILET_FAN_DELAY = 4;
const uint8_t TOILET_LIGHT_LEVEL = 5;
const uint8_t TOILET_LIGHT_AUTO = 6;

const uint8_t TOILET_PRESENCE = 0;
const uint8_t TOILET_TEMP = 1;
const uint8_t BATHROOM_TEMP = 2;
const uint8_t BATHROOM_HUMIDITY = 3;

const uint32_t PERIODICAL_TIMER_FREQUENCY = 300000; //300ms
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

const uint8_t INPUT_COUNT = 4;
const uint8_t HOLDING_COUNT = 7;

uint8_t outputState[8] = { LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW }; //{ BATH_FAN, TOILET_FAN, BATH_LIGHT, TOILET_LIGHT, BOILER, HEAT1, HEAT2, HEAT3 }
uint16_t inputRegister[INPUT_COUNT] = { 0, 0, 0, 0 }; //{ TOILET_PRESENCE, TOILET_TEMP, BATHROOM_TEMP, BATHROOM_HUMIDITY }
uint16_t holdingRegister[HOLDING_COUNT] = { 100, 1, 7, 1, 15, 100, 1 }; //{ BATH_LIGHT_LEVEL, BATH_FAN_AUTO, BATH_FAN_TOLERANCE, TOILET_FAN_AUTO, TOILET_FAN_DELAY, TOILET_LIGHT_LEVEL, TOILET_LIGHT_AUTO }

uint8_t pcfInputState[4] = { HIGH, HIGH, HIGH, HIGH };

uint16_t bathFanRuleCounter = 0;
uint8_t bathFanSleep = 6;//6s
uint8_t bathFanRuleInterval = 20;//6s
float bathFanEnvMin = 0.0;
float bathFanEnvMax = 0.0;
float bathFanMaxHumidity = 100.0;
float bathFanEnvMaxDecay = 0.0;
float bathFanEnvMinDecay = 0.0;
const float BATH_FAN_HALFLIFE_MAX = 604800.0;
const float BATH_FAN_HALFLIFE_MIN = 7200.0;
typedef enum {FAN_OFF, FAN_NOCHANGE, FAN_ON};
uint8_t bathFanLastState = FAN_OFF;

uint8_t toiletLightLastState = LOW;
uint8_t isToiletFanAutoOn = LOW;
uint16_t toiletFanRuleCounter = 0;

const uint8_t FAN_DELAY_MINUTE = 200;//ticks per minute for 300ms timer

const uint8_t PWM_OUTPUT_STEPS = 100;
const uint8_t PWM_OUTPUT_LOG[101] =
{ 100, 100, 97, 97, 94, 94, 91, 91, 88, 88, 85, 85, 82, 82, 79, 79, 76, 76, 74, 74, 71, 71, 69, 69, 66, 66, 63, 63, 61, 61, 59, 59,
  56, 56, 54, 54, 52, 52, 50, 50, 48, 48, 46, 46, 44, 44, 42, 42, 40, 40, 38, 38, 36, 36, 34, 34, 32, 32, 30, 30, 28, 28, 26, 26, 25, 25, 24, 24, 23, 23, 22, 22, 20, 20,
  19, 19, 17, 17, 16, 16, 15, 15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5
};

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton bathFanButton(INPUT1_PIN, true, false);
OneButton toiletFanButton(INPUT2_PIN, true, false);
OneButton bathLightButton(INPUT3_PIN, true, false);
OneButton toiletLightButton(INPUT4_PIN, true, false);
HardwareTimer *bathTimer;
HardwareTimer *toiletTimer;
OneWire ds(DS18B20_PIN);
PCF8574 pcf8574(PCF8574_I2C_ADDRESS);

void setOutput(uint8_t out) {
  uint8_t pin;
  uint8_t pcfPin;
  uint8_t value = outputState[out];
  switch (out) {
    case BATH_FAN:
      pin = BATH_FAN_OUTPUT_PIN;
      pcfPin = PCF_BATH_FAN_OUTPUT_PIN;
      break;
    case TOILET_FAN:
      pin = TOILET_FAN_OUTPUT_PIN;
      pcfPin = PCF_TOILET_FAN_OUTPUT_PIN;
      break;
    case BATH_LIGHT:
      pin = BATH_LIGHT_OUTPUT_PIN;
      pcfPin = PCF_BATH_LIGHT_OUTPUT_PIN;
      setBathLightLevel(value ? holdingRegister[BATH_LIGHT_LEVEL] : 0);
      break;
    case TOILET_LIGHT:
      pin = TOILET_LIGHT_OUTPUT_PIN;
      pcfPin = PCF_TOILET_LIGHT_OUTPUT_PIN;
      setToiletLightLevel(value ? holdingRegister[TOILET_LIGHT_LEVEL] : 0);
      break;
    case BOILER:
      pin = BOILER_OUTPUT_PIN;
      pcfPin = 254;
      break;
    case HEAT1:
      pin = HEAT1_OUTPUT_PIN;
      pcfPin = 254;
      break;
    case HEAT2:
      pin = HEAT2_OUTPUT_PIN;
      pcfPin = 254;
      break;
    case HEAT3:
      pin = HEAT3_OUTPUT_PIN;
      pcfPin = 254;
      break;            
  }
  digitalWrite(pin, value);
  if (pcfPin != 254) {
    pcf8574.digitalWrite(pcfPin, !value);
  }  
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastOutputState[8] = { outputState[BATH_FAN], outputState[TOILET_FAN], outputState[BATH_LIGHT], outputState[TOILET_LIGHT], outputState[BOILER], outputState[HEAT1], outputState[HEAT2], outputState[HEAT3] };

  if (address > 8 || (address + length) > 8) {
    return STATUS_ILLEGAL_DATA_ADDRESS;
  }
  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }
  for (uint16_t i = 0; i < 8; i++) {
    if (outputState[i] != lastOutputState[i]) {
      setOutput(i);
      if (i == TOILET_FAN) {
        isToiletFanAutoOn = LOW;
      }  
    }
  }
  return STATUS_OK;
}

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  if (address > 8 || (address + length) > 8) {
    return STATUS_ILLEGAL_DATA_ADDRESS;
  }
  for (uint16_t i = 0; i < length; i++) {
    slave.writeCoilToBuffer(i, outputState[address + i]);
  }
  return STATUS_OK;
}

void clickBathFanButton() {
  outputState[BATH_FAN] = !outputState[BATH_FAN];
  setOutput(BATH_FAN);
}

void clickToiletFanButton() {
  isToiletFanAutoOn = LOW;
  outputState[TOILET_FAN] = !outputState[TOILET_FAN];
  setOutput(TOILET_FAN);
}

void clickBathLightButton() {
  outputState[BATH_LIGHT] = !outputState[BATH_LIGHT];
  setOutput(BATH_LIGHT);
}

void clickToiletLightButton() {
  outputState[TOILET_LIGHT] = !outputState[TOILET_LIGHT];
  setOutput(TOILET_LIGHT);
}

void initButtons() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(INPUT3_PIN, INPUT);
  pinMode(INPUT4_PIN, INPUT);
  pinMode(INPUT5_PIN, INPUT);
  pinMode(BATH_FAN_OUTPUT_PIN, OUTPUT);
  pinMode(TOILET_FAN_OUTPUT_PIN, OUTPUT);
  pinMode(BATH_LIGHT_OUTPUT_PIN, OUTPUT);
  pinMode(TOILET_LIGHT_OUTPUT_PIN, OUTPUT);
  pinMode(BOILER_OUTPUT_PIN, OUTPUT);
  pinMode(HEAT1_OUTPUT_PIN, OUTPUT);
  pinMode(HEAT2_OUTPUT_PIN, OUTPUT);
  pinMode(HEAT3_OUTPUT_PIN, OUTPUT);
  pinMode(BATH_LIGHT_LEVEL_PIN, OUTPUT);
  pinMode(TOILET_LIGHT_LEVEL_PIN, OUTPUT);

  bathFanButton.attachClick(clickBathFanButton);
  toiletFanButton.attachClick(clickToiletFanButton);
  bathLightButton.attachClick(clickBathLightButton);
  toiletLightButton.attachClick(clickToiletLightButton);
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
  uint16_t rawTemperature = 0;
  uint8_t checksum = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_TEMP_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) {
    return I2C_ERROR;
  }

  delay(22);

  Wire.requestFrom(HTU21D_I2C_ADDRESS, 3);
  if (Wire.available() != 3) {
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
  }
  return humidity;
}

float readCompensatedHumidity(float temperature) {
  float humidity = readHumidity();
  if (humidity == I2C_ERROR || temperature == I2C_ERROR) {
    return I2C_ERROR;
  }
  if (humidity > 100) {
    setHTUHeater(true);
    humidity = 100;
  } else {
    setHTUHeater(false);
  }
  if (temperature > 0 && temperature < 80) {
    humidity = humidity + (25.0 - temperature) * HTU21D_TEMP_COEFFICIENT;
  }
  return humidity;
}

void setHTUHeater(int8_t isEnabled) {
  uint8_t userRegisterData = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_READ);
  Wire.endTransmission(true);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 1);
  if (Wire.available() != 1) {
    return;
  }
  userRegisterData = Wire.read();

  if (isEnabled) {
      userRegisterData |= HTU21D_HEATER_ON;
  } else {    
      userRegisterData &= HTU21D_HEATER_OFF;
  }
  
  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_WRITE);
  Wire.write(userRegisterData);
  Wire.endTransmission(true);
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

void processBathFanRule() {
  bathFanRuleCounter++;
  if (bathFanRuleCounter < bathFanRuleInterval) {
    return;
  }
  bathFanRuleCounter = 0;

  float humidity = inputRegister[BATHROOM_HUMIDITY];
  if (bathFanEnvMin > humidity) {
    bathFanEnvMin = humidity;
  } else {
    bathFanEnvMin = humidity - (humidity - bathFanEnvMin) * bathFanEnvMinDecay;
  }
  if (outputState[BATH_LIGHT]) {
    bathFanEnvMax = humidity;
    return;
  }
  if (bathFanEnvMax < humidity) {
    bathFanEnvMax = humidity;
  } else {
    bathFanEnvMax = humidity + (bathFanEnvMax - humidity) * bathFanEnvMaxDecay;
  }
  float thresholdLow = bathFanEnvMin + (bathFanEnvMax - bathFanEnvMin) / 8.0f;
  float thresholdHigh = bathFanEnvMin + (bathFanEnvMax - bathFanEnvMin) / 2.0f;

  int newState = FAN_NOCHANGE;
  if (humidity < thresholdLow) {
    newState = FAN_OFF;
  } else if (humidity > thresholdHigh) {
    newState = FAN_ON;
  }
  if (newState != bathFanLastState) {
    switch (newState) {
      case FAN_OFF:
        outputState[BATH_FAN] = LOW;
        setOutput(BATH_FAN);
        break;
      case FAN_ON:
        outputState[BATH_FAN] = HIGH;
        setOutput(BATH_FAN);
        break;
    }
  }
  bathFanLastState = newState;
}

void processToiletLightRule() {
  if (inputRegister[TOILET_PRESENCE] != toiletLightLastState) {
    outputState[TOILET_LIGHT] = inputRegister[TOILET_PRESENCE];
    setOutput(TOILET_LIGHT);
  }
  toiletLightLastState = inputRegister[TOILET_PRESENCE];
}

void processToiletFanRule() {
  if (outputState[TOILET_LIGHT]) {
    toiletFanRuleCounter = 0;
    if (isToiletFanAutoOn) {
      isToiletFanAutoOn = LOW;
      outputState[TOILET_FAN] = LOW;
      setOutput(TOILET_FAN);
    }  
  } else {
    if ((toiletFanRuleCounter == 0) && !outputState[TOILET_FAN]) {
       isToiletFanAutoOn = HIGH;
       outputState[TOILET_FAN] = HIGH;
       setOutput(TOILET_FAN);
       return;
    }
    if (outputState[TOILET_FAN]) {
      if (toiletFanRuleCounter < (holdingRegister[TOILET_FAN_DELAY] * FAN_DELAY_MINUTE)) {
        toiletFanRuleCounter++;
        return;
      }
      if (isToiletFanAutoOn) {
        isToiletFanAutoOn = LOW;
        outputState[TOILET_FAN] = LOW;
        setOutput(TOILET_FAN);
      }  
    }  
  }
}

void processRules() {
  if (holdingRegister[BATH_FAN_AUTO]) {
    processBathFanRule();
  } else {
    bathFanRuleCounter = 0;
  }
  if (holdingRegister[TOILET_LIGHT_AUTO]) {
    processToiletLightRule();
  }
  if (holdingRegister[TOILET_FAN_AUTO]) {
    processToiletFanRule();
  } else {
    toiletFanRuleCounter = 0;
  }
}

void getBathTempHumidity() {
  float bathroomTemp = readTemperature();
  inputRegister[BATHROOM_TEMP] = bathroomTemp * 10; 
  inputRegister[BATHROOM_HUMIDITY] = readCompensatedHumidity(bathroomTemp);
}

void updateSensors() {
  inputRegister[TOILET_PRESENCE] = !digitalRead(INPUT5_PIN);
  inputRegister[TOILET_TEMP] = readDS() * 10;

  getBathTempHumidity();

  delay(20);

  pcfInputState[BATH_FAN] = pcf8574.digitalRead(PCF_BATH_FAN_INPUT_PIN);
  pcfInputState[TOILET_FAN] = pcf8574.digitalRead(PCF_TOILET_FAN_INPUT_PIN);
  pcfInputState[BATH_LIGHT] = pcf8574.digitalRead(PCF_BATH_LIGHT_INPUT_PIN);
  pcfInputState[TOILET_LIGHT] = pcf8574.digitalRead(PCF_TOILET_LIGHT_INPUT_PIN);

  processRules();
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
  pcf8574.pinMode(PCF_BATH_FAN_INPUT_PIN, INPUT);
  pcf8574.pinMode(PCF_TOILET_FAN_INPUT_PIN, INPUT);
  pcf8574.pinMode(PCF_BATH_LIGHT_INPUT_PIN, INPUT);
  pcf8574.pinMode(PCF_TOILET_LIGHT_INPUT_PIN, INPUT);
  pcf8574.pinMode(PCF_BATH_FAN_OUTPUT_PIN, OUTPUT);
  pcf8574.pinMode(PCF_TOILET_FAN_OUTPUT_PIN, OUTPUT);
  pcf8574.pinMode(PCF_BATH_LIGHT_OUTPUT_PIN, OUTPUT);
  pcf8574.pinMode(PCF_TOILET_LIGHT_OUTPUT_PIN, OUTPUT);

  if (pcf8574.begin()) {
    pcf8574.digitalWrite(PCF_BATH_FAN_OUTPUT_PIN, HIGH);
    pcf8574.digitalWrite(PCF_TOILET_FAN_OUTPUT_PIN, HIGH);
    pcf8574.digitalWrite(PCF_BATH_LIGHT_OUTPUT_PIN, HIGH);
    pcf8574.digitalWrite(PCF_TOILET_LIGHT_OUTPUT_PIN, HIGH);
  }
}

void setBathLightLevel(uint16_t level) {
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(BATH_LIGHT_LEVEL_PIN), PinMap_PWM));
  bathTimer->setPWM(channel, BATH_LIGHT_LEVEL_PIN, PWM_FREQUENCY, PWM_OUTPUT_LOG[level]);
}

void setToiletLightLevel(uint16_t level) {
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(TOILET_LIGHT_LEVEL_PIN), PinMap_PWM));
  toiletTimer->setPWM(channel, TOILET_LIGHT_LEVEL_PIN, PWM_FREQUENCY, PWM_OUTPUT_LOG[level]);
}

/**
 * Handle Read Input Registers (FC=04)
 */
uint8_t readInputRegister(uint8_t fc, uint16_t address, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    slave.writeRegisterToBuffer(i, inputRegister[i + address]);
  }
  return STATUS_OK;
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
  uint8_t lastOutputLevel1 = holdingRegister[BATH_LIGHT_LEVEL];
  uint8_t lastOutputLevel2 = holdingRegister[TOILET_LIGHT_LEVEL];
  for (uint16_t i = 0; i < length; i++) {
    uint16_t value = slave.readRegisterFromBuffer(i);
    uint16_t index = i + address;
    if (
      (((index == BATH_LIGHT_LEVEL) || (index == TOILET_LIGHT_LEVEL)) && (value <= PWM_OUTPUT_STEPS)) ||
      (((index == BATH_FAN_AUTO) || (index == TOILET_FAN_AUTO) || (index == TOILET_LIGHT_AUTO)) && (value <= 1)) ||
      (((index == BATH_FAN_TOLERANCE) || (index == TOILET_FAN_DELAY)) && (value <= 100))
      ) {
      holdingRegister[index] = value;
    }  
  }
  if (holdingRegister[BATH_LIGHT_LEVEL] != lastOutputLevel1) {
    setBathLightLevel(holdingRegister[BATH_LIGHT_LEVEL]);
  }  
  if (holdingRegister[TOILET_LIGHT_LEVEL] != lastOutputLevel2) {
    setToiletLightLevel(holdingRegister[TOILET_LIGHT_LEVEL]);
  }  
  return STATUS_OK; 
}

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM3);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, MICROSEC_FORMAT);
  timer->attachInterrupt(updateSensors);
  timer->resume();
}

void initPWM() {
  TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(BATH_LIGHT_LEVEL), PinMap_PWM);
  bathTimer = new HardwareTimer(instance);
  TIM_TypeDef *instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(TOILET_LIGHT_LEVEL_PIN), PinMap_PWM);
  toiletTimer = new HardwareTimer(instance2);
}

void initBathData() {
  getBathTempHumidity();

  delay(bathFanSleep * 1000);

  float humidity = inputRegister[BATHROOM_HUMIDITY];

  bathFanEnvMin = humidity;
  bathFanEnvMax = humidity + holdingRegister[BATH_FAN_TOLERANCE];
  bathFanEnvMax = min(bathFanEnvMax, bathFanMaxHumidity);
  
  bathFanEnvMaxDecay = 1.0f * pow(2, -bathFanSleep / BATH_FAN_HALFLIFE_MAX);
  bathFanEnvMinDecay = 1.0f * pow(2, -bathFanSleep / BATH_FAN_HALFLIFE_MIN);
}

void setup() {
  initButtons();
  initI2C();
  initHTU21D();
  initDS();
  initPCF8574();
  initPWM();
  initBathData();
  initPeriodicalTimer();

  setBathLightLevel(0);
  setToiletLightLevel(0);
  
  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
  slave.cbVector[CB_READ_INPUT_REGISTERS] = readInputRegister;
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readHolding;
  slave.cbVector[CB_WRITE_HOLDING_REGISTERS] = writeHolding;
  
  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);

  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  bathFanButton.tick(pcfInputState[BATH_FAN] == LOW);
  toiletFanButton.tick(pcfInputState[TOILET_FAN] == LOW);
  bathLightButton.tick(pcfInputState[BATH_LIGHT] == LOW);
  toiletLightButton.tick(pcfInputState[TOILET_LIGHT] == LOW);
  
  slave.poll();
  
  IWatchdog.reload();
}
