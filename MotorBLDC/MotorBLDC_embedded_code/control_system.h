#ifndef __CONTROL_SYSTEM__
#define __CONTROL_SYSTEM__

// Basic period to schedule the control task,
// and also the minimum sample time.
#ifndef CONTROL_TASK_TS
#define CONTROL_TASK_TS 2
#endif

#include <reference_steps.h>

#include <cstddef>

// Number of controller internal states.
#ifndef CTRL_SYS_NC
#define CTRL_SYS_NC 6
#endif

// Number of filter internal states.
#ifndef CTRL_SYS_NF
#define CTRL_SYS_NF 1
#endif

// Number of plant manipulated inputs.
#ifndef CTRL_SYS_NU
#define CTRL_SYS_NU 1
#endif

// Number of plant output signals.
#ifndef CTRL_SYS_NY
// Four measurements: 
// (1) Motor position; (2) Motor speed; 
// (3) Box rotation angle around Z; 
// (4) Box rotation angle around X.
#define CTRL_SYS_NY 4  
#endif

// Number of reference signals in closed-loop experiments.
#ifndef CTRL_SYS_NR
#define CTRL_SYS_NR 1
#endif

// Number of internal parameters associated with 
// pre-defined control strategies.
#ifndef CTRL_SYS_NPARAMS
#define CTRL_SYS_NPARAMS 16
#endif

// Definition of indices in the vector of controller internal parameters.
#define CTRL_SYS_PARAM_KP 0
#define CTRL_SYS_PARAM_TI 1
#define CTRL_SYS_PARAM_TD 2

// The idea is to create a global object 
// whose memory footprint will be known 
// in compilation time such that, if there
// is not enough memory in the SoC (e.g. ESP32 or
// Arduino Mega), in the compilation process
// a warning and/or error will alert the 
// user.
class ControlSystem {

    private:
        // Sample time as a multiple of CONTROL_TASK_TS
        // This makes it possible to change the sample time 
        // without changing the control task periodic scheduling.
        unsigned int N_ts, pace_counter;

        // Controller state vector.
        float xc[CTRL_SYS_NC];

        // Measurement Filter state vector.
        float xf[CTRL_SYS_NF];

        // Reference signal vector.
        float ref[CTRL_SYS_NR];

        // Controller output vector = Plant input vector.
        float u[CTRL_SYS_NU];

        // Plant measurement output vector.
        float ym[CTRL_SYS_NY];

        // Internal parameters for pre-defined control strategies.
        // These parameters can be changed from the GUI by the user.
        float internal_param[CTRL_SYS_NPARAMS];

        // Last time control action was computed.
        float ctrl_last_t;
        // Last time measurement process function was called.
        float meas_last_t;

        // Internal functions that will be defined 
        // for different systems to be controlled.

        // Setup measurement processes.
        void setup_sensors(void);

        // Setup actuation processes.
        void setup_actuators(void);

        // Measurement process.
        void measure_signals(float t);

        // Actual control code. Declaration.
        void run_controller_code(float t);

        // Actuation process declaration.
        void command_actuators(void);

        // Internal controller code.
        // The idea is to use this ID value
        // to inform to the GUI in the PC what is
        // code implemented in the hardware.
        float ctrl_code_id;

    public:
        // Constructors. 
        //
        // Obs.: There is no copy constructor because I expect to 
        // have just one global object of this class called 'CtrlSys'.
        ControlSystem(void);
        
        // Destructor.
        ~ControlSystem(void);

        void Initialize(void);

        // Reference generation
        ReferenceSteps RefSteps;

        // Access to the internal variables:
        float get_ym(unsigned char n);
        float get_u(unsigned char n);
        float get_ref(unsigned char n);
        float get_ctrl_code_id(void);

        // Sample time:
        float get_sample_time(void);
        void set_sample_time(float new_ts);

        // Internal parameters:
        float get_internal_param(int n);
        void set_internal_param(int n, float val);

        // Control system initialization.
        void reset(void);

         // Actions when the control task is about to start.
        void on_start_task(void);

        // Actions to perform when the control task is about to stop.
        void on_stop_task(void);

        // Measure, compute references, compute the control action,
        // and command the actuators.
        int run(float t);
};

#endif
