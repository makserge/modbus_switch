#include <ModbusSlave.h>
#include <Wire.h>
#include <OneButton.h>
#include <IWatchdog.h>

#define SLAVE_ID 21
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA3
#define RS485_TX_PIN PA2
#define RS485_TX_ENABLE_PIN PB1

#define INPUT1_PIN PA4
#define INPUT2_PIN PA1
#define INPUT3_PIN PA7
#define INPUT4_PIN PA0

#define OUTPUT1_PIN PA6
#define OUTPUT2_PIN PA5
#define OUTPUT3_PIN PA9
#define OUTPUT4_PIN PA10

const uint8_t OUT1_STATE = 0;
const uint8_t OUT2_STATE = 1;
const uint8_t OUT3_STATE = 2;
const uint8_t OUT4_STATE = 3;

const uint16_t WATCHDOG_TIMEOUT = 10000000; //10s

uint8_t outputState[4] = { LOW, LOW, LOW, LOW }; //{ OUT1_STATE, OUT2_STATE, OUT3_STATE, OUT4_STATE }

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);
OneButton button2(INPUT2_PIN, true, false);
OneButton button3(INPUT3_PIN, true, false);
OneButton button4(INPUT4_PIN, true, false);

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(OUT1_STATE, outputState[OUT1_STATE]);
  slave.writeCoilToBuffer(OUT2_STATE, outputState[OUT2_STATE]);
  slave.writeCoilToBuffer(OUT3_STATE, outputState[OUT3_STATE]);
  slave.writeCoilToBuffer(OUT4_STATE, outputState[OUT4_STATE]);
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
  return STATUS_OK;
}

void initButtons() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(INPUT2_PIN, INPUT);
  pinMode(INPUT3_PIN, INPUT);
  pinMode(INPUT4_PIN, INPUT);
  pinMode(OUTPUT1_PIN, OUTPUT);
  pinMode(OUTPUT2_PIN, OUTPUT);
  pinMode(OUTPUT3_PIN, OUTPUT);
  pinMode(OUTPUT4_PIN, OUTPUT);

  button1.attachClick(clickButton1);
  button2.attachClick(clickButton2);
  button3.attachClick(clickButton3);
  button4.attachClick(clickButton4);
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

void setup() {   
  initButtons();

  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;

  Serial.setRx(RS485_RX_PIN);
  Serial.setTx(RS485_TX_PIN);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);

  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  button1.tick();
  button2.tick();
  button3.tick();
  button4.tick();
  slave.poll();
  IWatchdog.reload();
}
