#include <IWatchdog.h>

#define PIR_PIN PA0
#define IR_PIN PA9
#define OUTPUT_PIN PB1

const uint16_t PIR_OPEN_TIMEOUT = 600; //1 min
const uint8_t PIR_CLOSE_TIMEOUT = 150; //15s
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

uint16_t pirOpenTimer = 0;
uint8_t pirMovementClosed = LOW;
uint8_t pirClosedTimer = PIR_CLOSE_TIMEOUT;

void initIO() {
  pinMode(PIR_PIN, INPUT);
  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(OUTPUT_PIN, OUTPUT);
}

void initPIR() {
  uint8_t pirState;
  do {
    pirState = digitalRead(PIR_PIN);
    delay(100);
  }
  while (pirState);
}

void updateSensors() {
  uint8_t irState = digitalRead(IR_PIN);
  uint8_t pirState = digitalRead(PIR_PIN);
 
  if (irState) {
    pirMovementClosed = LOW;
    pirClosedTimer = PIR_CLOSE_TIMEOUT;
    if (pirState) {
      pirOpenTimer = PIR_OPEN_TIMEOUT;
      setOut(HIGH);
    } else {
      if (pirOpenTimer > 0) {
        pirOpenTimer--;
      } else {  
        setOut(LOW);
      }
    }
  } else {
    pirOpenTimer = PIR_OPEN_TIMEOUT;
    if (pirClosedTimer >0) {
      pirClosedTimer--;
      return;
    }
    if (pirMovementClosed) {
      return;
    }
    if (pirState) {
      pirMovementClosed = HIGH; 
    }
    setOut(pirState);
  }
}
 
void setOut(uint8_t state) {
  digitalWrite(OUTPUT_PIN, state);
}

void setup() {
  initIO();
  initPIR();
  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  updateSensors();
  delay(100);
  IWatchdog.reload();
}
