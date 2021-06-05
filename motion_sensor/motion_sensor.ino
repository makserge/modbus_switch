#include <Wire.h>
#include <VL53L0X.h>
#include <IWatchdog.h>

#define PIR_PIN PA0
#define OUTPUT_PIN PB1

#define I2C_SDA PA10
#define I2C_SCL PA9

const uint8_t PERIODICAL_TIMER_FREQUENCY = 1; //1HZ
const uint32_t WATCHDOG_TIMEOUT = 10000000; //10s

VL53L0X sensor;

void initPeriodicalTimer() {
  HardwareTimer *timer = new HardwareTimer(TIM1);
  timer->setOverflow(PERIODICAL_TIMER_FREQUENCY, HERTZ_FORMAT);
  timer->attachInterrupt(updateSensors);
  timer->resume();
}

void updateSensors() {
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  Serial.println(digitalRead(PIR_PIN));
}
 
void initI2C() {
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin();
}

void initToF() {
  sensor.setTimeout(500);
  if (sensor.init()) {
      // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
    sensor.startContinuous();
    return;
  }
  Serial.println("Failed to detect and initialize sensor!");
}

void setup() {
  Serial.begin(9600);
  
  pinMode(PIR_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, HIGH);
  
  initI2C();
  initToF();
  initPeriodicalTimer();

  IWatchdog.begin(WATCHDOG_TIMEOUT);
}

void loop() {
  digitalWrite(OUTPUT_PIN, HIGH);
  delay(100);
  digitalWrite(OUTPUT_PIN, LOW);
  delay(100);
  IWatchdog.reload();
}
