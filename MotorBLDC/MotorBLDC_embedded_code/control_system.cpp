#include "control_system.h"

#include <Arduino.h>

float ControlSystem::get_ym(unsigned char n) {
  return (n > CTRL_SYS_NY) ? 0.0f : ym[n];
}

float ControlSystem::get_u(unsigned char n) {
  return (n > CTRL_SYS_NU) ? 0.0f : u[n];
}

float ControlSystem::get_ref(unsigned char n) {
  return (n > CTRL_SYS_NR) ? 0.0f : ref[n];
}

float ControlSystem::get_ctrl_code_id(void) {
  return ctrl_code_id;
}

float ControlSystem::get_sample_time(void) {
  return (float) (CONTROL_TASK_TS*N_ts);
}

void ControlSystem::set_sample_time(float new_ts) {
    if (new_ts > CONTROL_TASK_TS) {

        // Find the nearest integer value.
        N_ts = (int) (new_ts/CONTROL_TASK_TS + 0.5f);

    }
    else
        N_ts = 1;
}

float ControlSystem::get_internal_param(int n) {
  if ((n > 0) && (n < CTRL_SYS_NPARAMS))
    return internal_param[n];
  
  // Specific value that will signal an error.
  return -1234.0f;
}

void ControlSystem::set_internal_param(int n, float val) {
  if ((n > 0) && (n < CTRL_SYS_NPARAMS))
    internal_param[n] = val;
}

ControlSystem::ControlSystem(void) {
  // Zero = not specified.
  // Any nonzero value will represent other possibilities.
  ctrl_code_id = 0.0f;

  // Sample time as an integer multiple of CONTROL_TASK_TS
  N_ts = 1;

  for (int n = 0; n < CTRL_SYS_NPARAMS; n++)
    internal_param[n] = 0.0f;

  reset();
}

ControlSystem::~ControlSystem(void) {
  on_stop_task();
}

void ControlSystem::Initialize(void) {
  setup_sensors();
  setup_actuators();
}

void ControlSystem::reset(void) {
  unsigned char n;

  // Internal pace counter to effect sample time change.
  pace_counter = 0;

  ctrl_last_t = 0.0f;
  meas_last_t = 0.0f;

  for (n = 0; n < CTRL_SYS_NC; n++)
    xc[n] = 0.0f;

  for (n = 0; n < CTRL_SYS_NU; n++)
    u[n] = 0.0f;

  for (n = 0; n < CTRL_SYS_NY; n++)
    ym[n] = 0.0f;

  for (n = 0; n < CTRL_SYS_NR; n++)
    ref[n] = 0.0f;

  for (n = 0; n < CTRL_SYS_NF; n++)
    xf[n] = 0.0f;
}

// Measurement, control law computation and actuation,
// in this order, if it is time to do so: when the number
// of times this function was called is equal to N_ts.
int ControlSystem::run(float t) 
{
  int retval = 0;

  pace_counter %= N_ts;

  if (pace_counter == 0) {

    measure_signals(t);
    meas_last_t = t;

    run_controller_code(t);
    command_actuators();
    ctrl_last_t = t;

    retval = 1;
  }
  pace_counter += 1;

  return retval;
}

/////////////////////////////////////////////////////////////////////
/// Include code specific to the Hardware Platform we are using. ////
/////////////////////////////////////////////////////////////////////
// This will also include the code for "generic controllers".
#include "MotorBLDC.h"
