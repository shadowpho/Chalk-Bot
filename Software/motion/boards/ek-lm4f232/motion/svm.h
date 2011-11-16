// svm.h - Definition file for interface for RC servo PWM, main update tick for system.
// Alex Suchko for ChalkBot
// November 10,2011

// Definitions

#define SVM_1           (1)           // hash definitions for accessing svm 1 through APIs
#define SVM_2           (2)           // hash definitions for accessing svm 2 through APIs

// Function Prototypes

void svm_init(void);                  // setup pins and PWM hardware to talk to SVM, provide main system
                                      // update tick
void svm_set_pulse(unsigned char ucSvm, unsigned long ulWidth);
                                      // update pulse width for given SVM output
void svm_set_us(unsigned char ucSvm, unsigned long ulMicroseconds);
                                      // update pulse width for given SVM output, in us

// EOF