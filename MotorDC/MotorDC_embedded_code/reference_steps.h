#ifndef __REFERENCE_STEPS__
#define __REFERENCE_STEPS__

#include <string.h>

// Maximum number of elements in the
// set of pre-programmed reference values
#define MAX_REF_STEPS 32

// Type of element in the vector of reference values.
typedef struct {
  float time;
  float refvalue;
} ref_step_t;


// Class and global object to take care
// of reference changes in steps.
class ReferenceSteps {
  public:
    ReferenceSteps(void);

    void reset(void);
    void store(float t, float value);

    void set(float t, float value);
    float get(float t);

  private:
    ref_step_t ref_steps[MAX_REF_STEPS];

    unsigned char ref_steps_size;
    unsigned char ref_step_current;

    void sorttime(void); 
};


#endif