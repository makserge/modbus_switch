#include <ModbusSlave.h>
#include <OneButton.h>

#define SLAVE_ID 20
#define RS485_BAUDRATE 9600

#define RS485_TX_ENABLE_PIN PA_0
#define INPUT1_PIN PA_5
#define OUTPUT1_PIN PB_0

const uint8_t OUT1_STATE = 0;

uint8_t outputState[1] = { LOW }; //{ OUT1_STATE }

Modbus slave(SLAVE_ID, RS485_TX_ENABLE_PIN);
OneButton button1(INPUT1_PIN, true, false);

uint8_t readDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  slave.writeCoilToBuffer(OUT1_STATE, outputState[OUT1_STATE]);
  return STATUS_OK;
}

/**
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
  uint8_t lastOutputState1 = outputState[OUT1_STATE];

  for (uint16_t i = 0; i < length; i++) {
    outputState[i + address] = slave.readCoilFromBuffer(i);
  }

  if (outputState[OUT1_STATE] != lastOutputState1) {
    setOutput(OUTPUT1_PIN, OUT1_STATE);
  } 
  return STATUS_OK;
}

void initButtons() {
  pinMode(INPUT1_PIN, INPUT);
  pinMode(OUTPUT1_PIN, OUTPUT);

  button1.attachClick(clickButton1);
}

void setOutput(uint8_t pin, uint8_t value) {
  digitalWrite(pin, value);
}

void clickButton1() {
  outputState[OUT1_STATE] = !outputState[OUT1_STATE];
  setOutput(OUTPUT1_PIN, OUT1_STATE);
}

void setup() {
  initButtons();

  slave.cbVector[CB_READ_COILS] = readDigitalOut;
  slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;

  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(RS485_BAUDRATE);
  slave.begin(RS485_BAUDRATE);
}

void loop() {
  button1.tick();
  slave.poll();
}
