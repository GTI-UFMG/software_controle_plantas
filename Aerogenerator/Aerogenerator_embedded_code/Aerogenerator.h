////////////////////////////////////////////////////////////
// DC motor with two channels encoder and gearbox, plus H-bridge LM298: hardware setup
////////////////////////////////////////////////////////////

#include <ESP32Encoder.h>
#include <esp32-hal-ledc.h>

// PWM config
#define PWM_BASE_FREQ 25000
#define PWM_RESOLUTION 8

// BLDC Cooler AFB0812SH Motor connection
#define PWM               14  // (INV_A) 
#define ENCODER_CHA       32  // From the generator optocoupler.  
#define ENCODER_CHB       35  // From the cooler motor optocoupler.
#define ENCODER_PPR       4   // Pulses Per Revolution (Encoder 11 PPR, vallue at output verified experimentally)
#define ADC_PIN           34  // Analog-to-Digital Covnerter pin
#define PWM_CH 1

// Encoder global object
ESP32Encoder motor_enc;

// Initialize and configure measurement processes.
void ControlSystem::setup_sensors(void) {
  // Setup encoder.
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  ESP32Encoder::isrServiceCpuCore = 0;

  // Configuration for the BLDC cooler motor:
	::motor_enc.attachSingleEdge(ENCODER_CHA,-1);
  ::motor_enc.clearCount();
}

// Initialize and configure actuation processes.
void ControlSystem::setup_actuators(void) {

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

  if (pwm < 0.0f)
    pwm = 0.0f;

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
//   xf           -> measurement filter state vector (these variables will be reset to zero right before the start of the control experiment).
// Measurement process.
void ControlSystem::measure_signals(float t) {
  float delta_pos, pos_motor;

  // Output voltage from the generator [mV]
  ym[0] = (float) analogRead(ADC_PIN)/4095.0*3300.0f;

  // Variation of angular position [revolutions] estimated from encoder accumulated pulses.
  // Hypothesis: PPR pulses per turn (revolution).
  pos_motor = motor_enc.getCount() / ((float) ENCODER_PPR);

  // Aerogenerator angular speed estimation in [revolutions/minute = RPM]:
  // Note that the angular speed is updated only after enough 
  // pulses have been received from the optocoupler sensor, or if a timeout 
  // has occurred (in case the speed is too low).
  // xf[0] is used to store the last time the angular speed was measured.
  // xf[1] is used to store the motor position at the last time the speed was updated.
  delta_pos = pos_motor - xf[1];
  if ((delta_pos > 2.0f) || ((t - xf[0]) > 0.1f)) {
    ym[1] = delta_pos / (t - xf[0])*60.0f;
    xf[0] = t;
    xf[1] = pos_motor;
  }

  // Not used.
  ym[2] = 0.0f;
  ym[3] = 0.0f;
}

// The next line will include the actual control law.
#include "control_strategy.h"