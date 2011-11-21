// hbridge.h - Definition file for interface with the ChalkBot hbridge modules.
// Alex Suchko for ChalkBot
// November 10,2011

// Definitions

#define HBR_LEFT          (1)         // hash code to refer to left hbridge
#define HBR_RIGHT         (2)         // hash code to refer to right hbridge

// Function Prototypes

void hbr_init(void);                  // setup pins and PWM hardware to talk to hbridge
void hbr_set_reset(unsigned char ucHbr, unsigned char new_state);
void hbr_set_pwml(unsigned char ucHbr, unsigned char new_state);
void hbr_set_phase(unsigned char ucHbr, unsigned char new_state);
void hbr_set_pulse(unsigned char ucHbr, unsigned long ulWidth);
void hbr_set_effort(unsigned char ucHbr, signed short ssEffort);


// EOF