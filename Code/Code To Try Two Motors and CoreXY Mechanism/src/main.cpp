// =====================================================================
//  CoreXY two-motor + kinematics TEST
//  Verifies driver comms, motor wiring, and CoreXY belt routing/direction.
//  Standalone: no HMI, no homing, no storage. Pinout matches the diagram.
//
//  Test order (watch the carriage):
//    1) Motor A only  -> should trace ONE clean 45 deg diagonal
//    2) Motor B only  -> should trace the OTHER 45 deg diagonal
//    3) +X / -X / +Y / -Y  -> straight horizontal / vertical lines
//    4) Square         -> the real proof that kinematics + wiring are right
// =====================================================================

#include <Arduino.h>
#include <TMCStepper.h>
#include "FastAccelStepper.h"

// ---- Driver A  (TMC2209 #1, UART address 0) ----
#define A_STEP   26
#define A_DIR    27
#define A_EN     25
// ---- Driver B  (TMC2209 #2, UART address 1) ----
#define B_STEP   33
#define B_DIR    32
#define B_EN     14
// ---- Shared TMC UART (Serial1) ----
#define TMC_TX    4      // -> 1k -> shared PDN node
#define TMC_RX   13      // -> shared PDN node

// ---- Mechanics ----
#define R_SENSE       0.11f
#define RMS_CURRENT   1000        // mA RMS (motor is 2.3A, but this driver tops ~1.2A)
#define MICROSTEPS    16
#define STEPS_PER_MM  80.0f       // GT2 2mm x 20T = 40mm/rev ; 3200/40 = 80
#define TEST_MM       50.0f       // size of test moves (mm) - keep small & safe
#define SPEED_MMS     60.0f       // mm/s
#define ACCEL_MMS2    500.0f      // mm/s^2

HardwareSerial TMCSerial(1);
TMC2209Stepper driverA(&TMCSerial, R_SENSE, 0b00);   // address 0
TMC2209Stepper driverB(&TMCSerial, R_SENSE, 0b01);   // address 1

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *sA = nullptr;
FastAccelStepper *sB = nullptr;

void configDriver(TMC2209Stepper &d, const char *tag) {
  d.begin();
  d.toff(5);
  d.rms_current(RMS_CURRENT);
  d.microsteps(MICROSTEPS);
  d.en_spreadCycle(false);   // stealthChop (quiet)
  d.pwm_autoscale(true);
  Serial.printf("  %s UART link: %s\n", tag,
                d.test_connection() == 0 ? "OK" : "FAILED  <-- check PDN/GND/wiring");
}

// CoreXY:  A = X + Y ,  B = X - Y    (boot position is treated as 0,0)
void moveTo(float x_mm, float y_mm) {
  long a = lroundf((x_mm + y_mm) * STEPS_PER_MM);
  long b = lroundf((x_mm - y_mm) * STEPS_PER_MM);
  sA->moveTo(a);
  sB->moveTo(b);
  while (sA->isRunning() || sB->isRunning()) delay(2);
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== CoreXY motion test ===");

  TMCSerial.begin(115200, SERIAL_8N1, TMC_RX, TMC_TX);
  configDriver(driverA, "Driver A");
  configDriver(driverB, "Driver B");

  engine.init();
  sA = engine.stepperConnectToPin(A_STEP);
  sB = engine.stepperConnectToPin(B_STEP);

  // setDirectionPin(pin, dirHighCountsUp) - flip the bool to reverse a motor in software
  sA->setDirectionPin(A_DIR, true);
  sB->setDirectionPin(B_DIR, true);
  sA->setEnablePin(A_EN);  sA->setAutoEnable(true);
  sB->setEnablePin(B_EN);  sB->setAutoEnable(true);

  uint32_t spd = (uint32_t)(SPEED_MMS  * STEPS_PER_MM);
  uint32_t acc = (uint32_t)(ACCEL_MMS2 * STEPS_PER_MM);
  sA->setSpeedInHz(spd);  sA->setAcceleration(acc);
  sB->setSpeedInHz(spd);  sB->setAcceleration(acc);

  sA->setCurrentPosition(0);
  sB->setCurrentPosition(0);
  Serial.println("Boot position = (0,0). Center the carriage before powering on!");
}

void loop() {
  float D = TEST_MM;

  Serial.println("1) Motor A only  -> expect ONE 45 deg diagonal");
  moveTo(D, D);   delay(800);  moveTo(0, 0);  delay(800);

  Serial.println("2) Motor B only  -> expect the OTHER 45 deg diagonal");
  moveTo(D, -D);  delay(800);  moveTo(0, 0);  delay(800);

  Serial.println("3) +X (right)");  moveTo(D, 0);   delay(700);
  Serial.println("   -X (left)");   moveTo(-D, 0);  delay(700);
  moveTo(0, 0); delay(500);
  Serial.println("   +Y (forward)");moveTo(0, D);   delay(700);
  Serial.println("   -Y (back)");   moveTo(0, -D);  delay(700);
  moveTo(0, 0); delay(700);

  Serial.println("4) Square (the real test)");
  moveTo(D, 0);  delay(300);
  moveTo(D, D);  delay(300);
  moveTo(0, D);  delay(300);
  moveTo(0, 0);  delay(300);

  Serial.println("--- cycle complete, pausing 3s ---\n");
  delay(3000);
}
