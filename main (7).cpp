#include "ThisThread.h"
#include "mbed.h"

bool run = false; // whether or not motors are running

const float max_speed = 1.0; // lower this if robot is too fast

// LDR inputs and calibration
AnalogIn L_ldr(PC_1);       // Left LDR
DigitalOut L_ldr_gnd(PC_0); // ground for L LDR
AnalogIn R_ldr(PF_10);      // Right LDR
DigitalOut R_ldr_gnd(PF_9); // ground for R LDR

float L_ldr_sense = 0.0;
float R_ldr_sense = 0.0;

bool calibrate_ldr = true;
// bounds LDR for calibration
float min_L_ldr = 1.0;
float max_L_ldr = 0.0;
float min_R_ldr = 1.0;
float max_R_ldr = 0.0;

// L motor direction control
DigitalOut in1(PF_5); // L239D in1
DigitalOut in2(PF_3); // L239D in2

// R motor direction control
DigitalOut in3(PF_1);  // L239D in3
DigitalOut in4(PC_15); // L239D in4

// motor speed control
PwmOut L_motor_pwm(PF_6); // L239D en 1,2
PwmOut R_motor_pwm(PA_3); // L239D en 3,4

Timeout calibration_delay; // delay to calibrate LDRs before starting motors
Ticker motor_ticker;       // motor speeds are set in this timer interrupt

InterruptIn user_button(BUTTON1); // toggle motors on/off

void SetMotorSpeedISR() {
  L_motor_pwm.write(1.0 - (R_ldr_sense * max_speed));
  R_motor_pwm.write(1.0 - (L_ldr_sense * max_speed));
}

void MotorsOnOffISR() {
  if (run) {
    motor_ticker.detach();
    L_motor_pwm.write(0);
    R_motor_pwm.write(0);
    run = false;
  } else {
    motor_ticker.attach(&SetMotorSpeedISR, 20ms);
    run = true;
    calibrate_ldr = false;
  }
}

int main() {
  L_ldr.set_reference_voltage(3.0f);
  L_ldr_gnd = 0;
  R_ldr.set_reference_voltage(3.0f);
  R_ldr_gnd = 0;

  // set L motor forward direction
  in1 = 1;
  in2 = 0;

  // set R motor forward direction
  in3 = 0;
  in4 = 1;

  R_motor_pwm.period(0.001f); // 1khz
  L_motor_pwm.period(0.001f); // 1khz

  user_button.fall(&MotorsOnOffISR);

  while (true) {
    L_ldr_sense = L_ldr.read();
    R_ldr_sense = R_ldr.read();
    if (calibrate_ldr) {
      min_L_ldr = min(min_L_ldr, L_ldr_sense);
      max_L_ldr = max(max_L_ldr, L_ldr_sense);
      min_R_ldr = min(min_R_ldr, R_ldr_sense);
      max_R_ldr = max(max_R_ldr, R_ldr_sense);
    }
    // normalize LDR readings
    L_ldr_sense = (L_ldr_sense - min_L_ldr) / (max_L_ldr - min_L_ldr);
    R_ldr_sense = (R_ldr_sense - min_R_ldr) / (max_R_ldr - min_R_ldr);

    printf("L:%.6f,R:%.6f\n", L_ldr_sense, R_ldr_sense);
    ThisThread::sleep_for(20ms);
  }
}

