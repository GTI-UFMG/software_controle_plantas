///////////////////////////////////////////////////////////////////
// The control strategy is implemented here.
// Variables that can be used: 
//   t             -> time [s]
//   ctrl_last_t   -> last time this code run [s].
//   ym            -> vector of measured signals.
//   ref           -> vector of reference signals.
//   u             -> vector of input signals.
//   xc            -> controller state vector (initialized with zero values).
//
// The following text file has the actual C++ language implementation
// of the full control strategy.
//
// This file can be rewritten by the GUI depending on the user choice. 
//
int ControlSystem::custom_ctrl_code(float t)
{
    #include "control_strategy.txt"

    return 1;
}
