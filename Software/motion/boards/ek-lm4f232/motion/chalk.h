// chalk.h - Definition file for chalk mechanism controller.
// Alex Suchko for ChalkBot
// November 27,2011

// Definitions

#define CHALK_SVM_Z           (SVM_1)               // vertical chalk actuation servo - moves chalk tube up and down
#define CHALK_SVM_CLAMP       (SVM_2)               // chalk clamp actuation servo - clamps chalk into chalk tube

defined positions here...

// Function Prototypes

void chalk_init(void);                              // initializes underlying svm driver (see "svm.h") and chalk mechanism state variables

// EOF