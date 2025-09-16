#include "reference_steps.h"

// Class ReferenceSteps definition.

ReferenceSteps::ReferenceSteps(void)
{
  reset();
}

void ReferenceSteps::reset(void)
{
  int count;

  // Initialize the set of reference step values.
  for(count = 0; count < MAX_REF_STEPS; count++) {
    ref_steps[count].time = 0.0;
    ref_steps[count].refvalue = 0.0;
  }
  ref_steps_size = 0;
  ref_step_current = 0;
}

void ReferenceSteps::set(float t, float value)
{
  // Prepare to change the set point 0.2s after receiving the command.
  ref_steps[0].time = t;
  ref_steps[0].refvalue = ref_steps[ref_step_current].refvalue;
  ref_steps[1].time = t + 0.2f;
  ref_steps[1].refvalue = value;
  ref_step_current = 0;
  ref_steps_size = 2;
}


void ReferenceSteps::store(float t,float value)
{
  if (ref_step_current < MAX_REF_STEPS)
  {
    ref_steps[ref_steps_size].time = t;
    ref_steps[ref_steps_size].refvalue = value;
    ref_steps_size++;
  } 

  sorttime();
}


// Bubble sort...
void ReferenceSteps::sorttime(void)
{
  ref_step_t aux;

  unsigned char i, go = 1;

  while(go) {
    go = 0;
    for(i=0; i < ref_steps_size-1; i++){
      if (ref_steps[i+1].time < ref_steps[i].time)
      {
          memcpy(&aux,&ref_steps[i],sizeof(ref_step_t));
          memcpy(&ref_steps[i],&ref_steps[i+1],sizeof(ref_step_t));
          memcpy(&ref_steps[i+1],&aux,sizeof(ref_step_t));
          go++;
      }
    }
  }
}


// Return the current value (at the current time) of the reference step value.
float ReferenceSteps::get(float t)
{
  if (t < ref_steps[0].time)
    return 0.0f;

  // Assuming reference values are properly sorted in time.
  for(;(ref_step_current < (ref_steps_size-1)) && (t >= ref_steps[ref_step_current+1].time);ref_step_current++);

  return ref_steps[ref_step_current].refvalue;
}





