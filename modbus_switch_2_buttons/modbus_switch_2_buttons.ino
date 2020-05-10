#include <ModbusSlave.h>
#include <OneButton.h>

#define SLAVE_ID 21
#define RS485_BAUDRATE 9600

#define RS485_RX_PIN PA10
#define RS485_TX_PIN PA9
#define RS485_TX_ENABLE_PIN PA_7

#define INPUT1_PIN PA_0
#define INPUT2_PIN PA_1

#define OUTPUT1_PIN PB_0
#define OUTPUT2_PIN PB_1

const uint8_t OUT1_STATE = 0;
const uint8_t OUT2_STATE = 1;

uint8_t outputState[2] = { LOW, LOW }; //{ OUT1_STATE, OUT2_STATE }

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);
OneButton button2(INPUT2_PIN, true, false);

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
    setOutput(OUTPUT1_PIN, OUT1_STATE);
  }
  if (outputState[OUT2_STATE] != lastOutputState2) {
    setOutput(OUTPUT2_PIN, OUT2_STATE);
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

  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;

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
