////////////////////////////////////////////////////////////
// DC motor with two channels encoder and gearbox, plus H-bridge LM298: hardware setup
////////////////////////////////////////////////////////////

#include <ESP32Encoder.h>
#include <esp32-hal-ledc.h>

// PWM config
#define PWM_BASE_FREQ 25000
#define PWM_RESOLUTION 8

// LM298 dual half-bridge power driver and DC Motor encoder connection
#define PWM 18          // White wire  (PWM)
#define DIR_1 22        // Blue wire  (clockwise)
#define DIR_2 23        // Purple Wire (counterclockwise)
#define ENCODER_CHA 34  // Green wire (Signal A)
#define ENCODER_CHB 35  // Yellow wire (Signal B)
#define ENCODER_PPR 895 // Pulses Per Revolution (Encoder 11 PPR, vallue at output verified experimentally)
#define PWM_CH 1

// Encoder global object
ESP32Encoder motor_enc;

// Initialize and configure measurement processes.
void ControlSystem::setup_sensors(void) {
  // Setup encoder.
  // ESP32Encoder::useInternalWeakPullResistors = puType::none;
  ESP32Encoder::isrServiceCpuCore = 0;

  // Configuration for the DC motor.
  ::motor_enc.attachFullQuad(ENCODER_CHA, ENCODER_CHB);
  ::motor_enc.clearCount();
}

// Initialize and configure actuation processes.
void ControlSystem::setup_actuators(void) {
  // DC Brushless NIDEC motor configuration.
  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);

  // Initial direction (clockwise)
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, LOW);

  // PWM channel configuration (using the ledc library).
  ledcSetClockSource(LEDC_AUTO_CLK);
  ledcAttachChannel(PWM, PWM_BASE_FREQ, PWM_RESOLUTION, PWM_CH);

  u[0] = 0.0f;
  command_actuators();
}

// Actuation process.
// Available variables: controller output vector u.
void ControlSystem::command_actuators(void) {
  float pwm = u[0];

  if (pwm < 0) {
    digitalWrite(DIR_1, LOW);
    digitalWrite(DIR_2, HIGH);
    pwm = -pwm;
  } else {
    digitalWrite(DIR_1, HIGH);
    digitalWrite(DIR_2, LOW);
  }

  // Saturation:
  pwm = pwm > ((float)(1 << PWM_RESOLUTION) - 1.0f) ? ((float)(1 << PWM_RESOLUTION) - 1.0f) : pwm;
 
  // PWM generation.
  ledcWriteChannel(PWM_CH, (int)pwm);
}

// Actions when control task is about to start.
void ControlSystem::on_start_task(void) {
  // Reset the control system. It set internal states
  // (controller and measurement filter states) to zero, as well as
  // the ctrl_last_t and meas_last_t time instants.
  reset();

  // Start the encoder counter at zero.
  motor_enc.clearCount();
}

// Actions performed when the control task is about to stop.
void ControlSystem::on_stop_task(void) {
  // Initialize the reference steps values to zero.
  RefSteps.reset();

  // Stop the DC motor.
  u[0] = 0.0;
  command_actuators();
}

///////////////////////////////////////////////////////////////////
// The measurement process is implemented here.
// Available variables (all private variables of the ControlSystem class,
// but especially the following):
//   t            -> time [s]
//   meas_last_t  -> last time this code run [s].
//   ym           -> vector of measured signals.
//   xf           -> measurement filter state vector.
// Measurement process.
void ControlSystem::measure_signals(float t) {
  float pos_motor;

  // Position [deg] estimation from encoder accumulated pulses.
  // Hypothesis: PPR pulses per turn (revolution).
  pos_motor = motor_enc.getCount() / ((float) ENCODER_PPR)*(360.0f);

  // DC Motor angular speed estimation in [deg/s]:
  ym[1] = (pos_motor - ym[0]) / (t - meas_last_t);

  // Update the motor angular position measurement in [deg]:
  ym[0] = pos_motor;

  // Euler angles from the IMU.
  ym[2] = 0.0;
  ym[3] = 0.0;
}

// The next line will include the actual control law.
#include "control_strategy.h"