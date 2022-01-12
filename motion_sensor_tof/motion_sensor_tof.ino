#include <Wire.h>
#include <IWatchdog.h>

#include "VL53L1X.h"

#define PIR_PIN PA0
#define OUTPUT_PIN PB1

const uint16_t TOF_THRESHOLD = 2000;

const uint16_t PIR_OPEN_TIMEOUT = 600; //1 min
const uint8_t PIR_CLOSE_TIMEOUT = 150; //15s
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
  Wire.begin();
  Wire.setClock(400000);

  tofSensor.setTimeout(500);
  if (!tofSensor.init()) {
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  tofSensor.setDistanceMode(VL53L1X::Long);
  tofSensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  tofSensor.startContinuous(50);  
}

void updateSensors() {
  uint8_t pirState = digitalRead(PIR_PIN);
  uint16_t tofDistance = tofSensor.read(true);
 
  if (tofDistance < TOF_THRESHOLD) {
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
  initTOF();
  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  updateSensors();
  delay(100);
  IWatchdog.reload();
}
