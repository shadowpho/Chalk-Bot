// chalk.h - Definition file for chalk mechanism controller.
// Alex Suchko for ChalkBot
// November 27,2011

// Definitions

#define CHALK_SVM_Z           (SVM_1)               // vertical chalk actuation servo - moves chalk tube up and down
#define CHALK_SVM_CLAMP       (SVM_2)               // chalk clamp actuation servo - clamps chalk into chalk tube

// Type definitions

typedef enum {                                      // state variable type for chalk machine
  SERVOS_OFF = 0,                                   // servos offline state
  MANUAL_MODE,                                      // servos under remote manual control
  AUTO_STORING_LIFT,                                // storing - lift
  AUTO_STORING_UNCLAMP,                             // storing - unclamp
  AUTO_STORING_LOWER,                               // storing - lower
  AUTO_STORING_CLAMP,                               // storing - clamp
  AUTO_STORING_LIFT2,                               // storing - lift2
  AUTO_IDLE,                                        // auto mode in steady state (watching for low chalk)
  AUTO_LOADING_LOWER,                               // loading - lower (assume start from store position)
  AUTO_LOADING_UNCLAMP,                             // loading - unclamp chalk
  AUTO_LOADING_LIFT,                                // loading - lift
  AUTO_LOADING_CLAMP,                               // loading - clamp
  AUTO_LOADING_LIFT2,                               // loading - lift2
  AUTO_CHALK_LOW                                    // chalk low detected while in idle - if load signal from here 
} chalk_machine_state_t;

// Function Prototypes

void chalk_init(void);                              // initializes underlying svm driver (see "svm.h") and chalk mechanism state variables
tBoolean chalk_sw_status(void);                     // reads the chalk switch status, true for depressed

// EOF