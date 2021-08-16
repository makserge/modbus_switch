#include "ModbusSlave.h"
#include "OneButton.h"

#define SLAVE_ID 40
#define RS485_BAUDRATE 9600

#define RS485_TX_ENABLE_PIN PA0
#define RS485_RX_PIN PA10
#define RS485_TX_PIN PA9

#define INPUT1_PIN PA5
#define OUTPUT1_PIN PB1

const uint8_t OUT1_STATE = 0;
const uint8_t OUT1_LONG_PRESS_STATE = 1;

uint8_t outputState[2] = { LOW, LOW }; //{ OUT1_STATE, OUT1_LONG_PRESS_STATE }

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, true);

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  if (address > 2 || (address + length) > 2) {
    return STATUS_ILLEGAL_DATA_ADDRESS;
  }
  for (uint16_t i = 0; i < length; i++) {
    slave.writeCoilToBuffer(i, outputState[address + i]);
  }
  return STATUS_OK;
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastOutputState1 = outputState[OUT1_STATE];

  if (address > 2 || (address + length) > 2) {
    return STATUS_ILLEGAL_DATA_ADDRESS;
  }
  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  if (outputState[OUT1_STATE] != lastOutputState1) {
    setOutput(OUTPUT1_PIN, outputState[OUT1_STATE]);
  } 
  return STATUS_OK;
}

void clickButton1() {
  outputState[OUT1_STATE] = !outputState[OUT1_STATE];
  setOutput(OUTPUT1_PIN, outputState[OUT1_STATE]);
}

void longPressStart1() {
  outputState[OUT1_LONG_PRESS_STATE] = HIGH;
}

void longPressStop1() {
  outputState[OUT1_LONG_PRESS_STATE] = LOW;
}

void initButtons() {
  pinMode(OUTPUT1_PIN, OUTPUT);

  button1.attachClick(clickButton1);
  button1.attachLongPressStart(longPressStart1);
  button1.attachLongPressStop(longPressStop1);
}

void setOutput(uint8_t pin, uint8_t value) {
  digitalWrite(pin, value);
}

void setup() {
  initButtons();

  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;

  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);
}

void loop() {
  button1.tick();
  slave.poll();
}
