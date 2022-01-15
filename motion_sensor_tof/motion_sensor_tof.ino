#include <Wire.h>
#include <IWatchdog.h>

#include "VL53L1X.h"

#define PIR_PIN PA0
#define OUTPUT_PIN PB1

const uint16_t TOF_THRESHOLD = 1300;

const uint16_t PIR_OPEN_TIMEOUT = 200; //1 min
const uint8_t PIR_CLOSE_TIMEOUT = 50; //15s
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

uint16_t pirOpenTimer = 0;
uint8_t pirMovementClosed = LOW;
uint8_t pirClosedTimer = PIR_CLOSE_TIMEOUT;

VL53L1X tofSensor;

void initIO() {
  pinMode(PIR_PIN, INPUT);
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

void initTOF() {
  Wire.setSDA(PA10);
  Wire.setSCL(PA9);
  Wire.begin();
  Wire.setClock(400000);

  tofSensor.setTimeout(500);
  if (!tofSensor.init()) {
    while (1);
  }
  
  tofSensor.setDistanceMode(VL53L1X::Long);
  tofSensor.setMeasurementTimingBudget(50000);
  tofSensor.startContinuous(50);  
}

void updateSensors() {
  uint8_t pirState = digitalRead(PIR_PIN);
  uint16_t tofDistance = tofSensor.read(true);
  if (tofDistance > TOF_THRESHOLD) {
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
    if (pirMovementClosed) {
      return;
    }
    if (pirClosedTimer >0) {
      pirClosedTimer--;
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
  initTOF();
  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  updateSensors();
  delay(300);
  IWatchdog.reload();
}
