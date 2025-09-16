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

float ControlSystem::get_internal_param(unsigned int n) {
  if ((n >= 0) && (n < CTRL_SYS_NPARAMS))
    return internal_param[n];
  
  // Specific value that will signal an error.
  return -1234.0f;
}

void ControlSystem::set_internal_param(unsigned int n, float val) {
  if ((n >= 0) && (n < CTRL_SYS_NPARAMS))
    internal_param[n] = val;
}

void ControlSystem::set_ctrl_code(unsigned int code) {
  ctrl_code_id = code;
}

void ControlSystem::set_ctrl_variable(unsigned int option) {
  ctrl_variable = option;
}

void ControlSystem::set_deadzone_comp(float cn, float cp) {
  deadzone_cn = cn;
  deadzone_cp = cp;
}

ControlSystem::ControlSystem(void) {
  unsigned char n;
  
  // Controller strategy: initially = open loop.
  ctrl_code_id = CTRL_OPEN_LOOP;

  // Sample time as an integer multiple of CONTROL_TASK_TS
  N_ts = 1;

  for (n = 0; n < CTRL_SYS_NPARAMS; n++)
    internal_param[n] = 0.0f;

  // No deadzone compensation.
  deadzone_cn = 0.0f;
  deadzone_cp = 0.0f;

  // Set controlled variable as 'speed'.
  ctrl_variable = CTRL_VAR_SPEED;

  reset();
}

ControlSystem::~ControlSystem(void) {
  on_stop_task();
}

// Prepare sensors and actuators.
void ControlSystem::Initialize(void) {
  setup_sensors();
  setup_actuators();
}

// Reset internal signals and the sampling time.
// Warning: This code do not change the controlled variable
// nor the controller strategy, nor the controller parameters.  
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

    // Get the reference value:
    ref[0] = RefSteps.get(t);

    retval = run_controller_code(t);
    if (retval) {
      // Effectively changes the control action
      // by introducing the dead-zone compensation.
      if (u[0] > 0.0f)
        u[0] += deadzone_cp;
      else
        if (u[0] < 0.0f)
          u[0] += deadzone_cn;
    }
    else
      u[0] = 0.0f;
      
    command_actuators();
    ctrl_last_t = t;

  }
  pace_counter += 1;

  return retval;
}

/////////////////////////////////////////
// Pre-programmed Controller Strategies: 
////////////////////////////////////////

int ControlSystem::ctrl_open_loop(float t) 
{
  // Connects reference to input directly:
  u[0] = ref[0];

  return 1;
}

int ControlSystem::ctrl_pid_ct(float t)
{
  // Ensaio em malha Fechada: usando PID especificado a partir 
  // de Kp, Ti e Td, e implementado usando:
  //
  // Acao integral = transformacao bilinear (ou regra do trapezio)
  // Acao derivativa = Euler de um passo-a-frente (assim garante-se 
  //                   atenuacao em altas frequencias mais efetiva do
  //                   que o que se obtem via transformacao bilinear)

  float kp = internal_param[CTRL_SYS_PARAM_KP]; 
  float Ti = internal_param[CTRL_SYS_PARAM_TI];
  float Td = internal_param[CTRL_SYS_PARAM_TD];

  // Actual Sample time:
  float ts;

  // Auxiliary variables:
  float e;

  ts = t - ctrl_last_t;

  if (ctrl_variable == CTRL_VAR_POSITION)
    e = ref[0] - ym[0];
  else
    e = ref[0] - ym[1];

  // Compute the output:
  if (Ti > 0.0f)
    u[0] = kp*(e + 1/Ti*(ts/2.0f*e + xc[0]) + Td/ts*(e - xc[1]));
  else
    u[0] = kp*(e + Td/ts*(e - xc[1]));

  // Update controller internal states:
  xc[0] = xc[0] + ts*e;
  xc[1] = e;

  return 1;
}

int ControlSystem::run_controller_code(float t) 
{
  int retval = 0;

  switch(ctrl_code_id) {

    case CTRL_OPEN_LOOP:
      return ctrl_open_loop(t);
    break;

    case CTRL_PID_CT:
      return ctrl_pid_ct(t);
    break;

    default: // Do nothing.
      u[0] = 0.0;
      return 1;
  }

  return retval;
}

/////////////////////////////////////////////////////////////////////
/// Include code specific to the Hardware Platform we are using. ////
/////////////////////////////////////////////////////////////////////
// This will also include the code for "generic controllers".
#include "MotorBLDC.h"
